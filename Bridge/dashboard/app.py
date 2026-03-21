#!/usr/bin/env python3
"""
Tesla CAN Dashboard - Récepteur UDP + Décodeur DBC + Dashboard Temps Réel
Reçoit les trames CAN depuis l'ESP32 WiFi AP (bridge) via UDP port 5555
Décode les signaux avec le fichier tesla_can.dbc
Affiche un dashboard web Plotly Dash avec tableau + graphiques temps réel
"""

import struct
import socket
import threading
import time
import os
import sys
import subprocess
import platform
import atexit
import signal
from datetime import datetime
from collections import deque, defaultdict
from pathlib import Path

import dash
from dash import dcc, html, dash_table, callback_context
from dash.dependencies import Input, Output, State
import dash_bootstrap_components as dbc
import plotly.graph_objs as go
import cantools
import can

# =============================================================================
# CONFIGURATION
# =============================================================================

# WiFi AP settings
WIFI_SSID = "bridge"
WIFI_PASSWORD = "teslamodel3"
ESP32_IP = "192.168.4.1"

# UDP settings
UDP_PORT = 5556
UDP_BIND_IP = "0.0.0.0"

# Dashboard
DASH_HOST = "0.0.0.0"
DASH_PORT = 8080
UPDATE_INTERVAL_MS = 500  # Dashboard refresh rate (ms) — 500ms for 84 outputs cockpit

# Data retention: 2 minutes sliding window
WINDOW_SECONDS = 120
MAX_POINTS = WINDOW_SECONDS * 100  # ~100 msgs/sec max

# UDP packet format
UDP_MAGIC = b'\xCA\x4E'
UDP_PACKET_SIZE = 18

# DBC file path (relative to this script)
SCRIPT_DIR = Path(__file__).parent
DBC_PATH = SCRIPT_DIR.parent / "tesla_can.dbc"

# =============================================================================
# GLOBAL STATE
# =============================================================================

# Connection health thresholds
CONN_TIMEOUT_STALE = 3.0       # Seconds without packet -> "stale"
CONN_TIMEOUT_LOST = 10.0       # Seconds without packet -> "lost"
WIFI_CHECK_INTERVAL = 5.0      # Seconds between WiFi checks
PING_INTERVAL = 10.0           # Seconds between ESP32 pings
RECONNECT_BACKOFF_MAX = 30.0   # Max seconds between reconnect attempts


class AppState:
    """Thread-safe application state."""
    def __init__(self):
        self.lock = threading.Lock()
        # Raw message log (deque with max length)
        self.messages = deque(maxlen=5000)
        # Decoded signals: {signal_name: deque of (timestamp, value)}
        self.signals = defaultdict(lambda: deque(maxlen=MAX_POINTS))
        # Last known values: {signal_name: (timestamp, value, unit, msg_name)}
        self.last_values = {}
        # Counters
        self.rx_count = 0
        self.decode_errors = 0
        self.sequence_errors = 0
        self.last_sequence = -1
        self.start_time = time.time()
        # WiFi status
        self.wifi_connected = False
        self.wifi_reconnecting = False
        self.wifi_rssi = None          # dBm if available
        self.wifi_reconnect_count = 0
        self.wifi_last_check = 0
        self.wifi_ssid_current = None
        # UDP / connection health
        self.udp_active = False
        self.last_packet_time = 0      # Timestamp of last received UDP packet
        self.packet_rate = 0.0         # Messages per second (rolling)
        self.ping_latency_ms = None    # Ping to ESP32 (ms)
        self.ping_reachable = False
        self._rate_window = deque(maxlen=200)  # Timestamps for rate calc
        # Connection state: 'disconnected' | 'wifi_only' | 'receiving' | 'stale'
        self.connection_state = 'disconnected'
        # BLF recording
        self.recording = False
        self.blf_writer = None
        self.blf_path = None
        self.record_count = 0

    def update_connection_state(self):
        """Derive the overall connection state from current metrics.
        Must be called while holding self.lock."""
        now = time.time()
        if not self.wifi_connected:
            self.connection_state = 'disconnected'
        elif self.last_packet_time == 0:
            self.connection_state = 'wifi_only'
        elif (now - self.last_packet_time) > CONN_TIMEOUT_LOST:
            self.connection_state = 'stale'
        elif (now - self.last_packet_time) > CONN_TIMEOUT_STALE:
            self.connection_state = 'stale'
        else:
            self.connection_state = 'receiving'

    def record_packet_time(self, ts):
        """Record a received packet timestamp and recalculate rate.
        Must be called while holding self.lock."""
        self.last_packet_time = ts
        self._rate_window.append(ts)
        # Calculate rate over the last second of packets
        cutoff = ts - 1.0
        while self._rate_window and self._rate_window[0] < cutoff:
            self._rate_window.popleft()
        self.packet_rate = len(self._rate_window)


state = AppState()

# =============================================================================
# DBC LOADER
# =============================================================================

def load_dbc(path):
    """Load and return the DBC database."""
    if not path.exists():
        print(f"[ERREUR] Fichier DBC introuvable: {path}")
        sys.exit(1)
    try:
        db = cantools.database.load_file(str(path))
        print(f"[DBC] Chargé: {path.name}")
        print(f"[DBC] {len(db.messages)} messages, "
              f"{sum(len(m.signals) for m in db.messages)} signaux")
        return db
    except Exception as e:
        print(f"[ERREUR] Impossible de charger le DBC: {e}")
        sys.exit(1)

dbc_db = load_dbc(DBC_PATH)

# Build lookup: CAN ID -> message definition
msg_by_id = {}
for msg in dbc_db.messages:
    msg_by_id[msg.frame_id] = msg

# =============================================================================
# WiFi AUTO-CONNECT (macOS)
# =============================================================================

def check_wifi_connection():
    """Check if connected to the 'bridge' WiFi network.
    Returns (connected: bool, ssid: str|None, rssi: int|None)."""
    try:
        if platform.system() == "Darwin":
            result = subprocess.run(
                ["/System/Library/PrivateFrameworks/Apple80211.framework/"
                 "Versions/Current/Resources/airport", "-I"],
                capture_output=True, text=True, timeout=5
            )
            ssid = None
            rssi = None
            for line in result.stdout.splitlines():
                stripped = line.strip()
                if stripped.startswith("SSID:") and not stripped.startswith("BSSID"):
                    ssid = stripped.split(": ", 1)[-1]
                if stripped.startswith("agrCtlRSSI:"):
                    try:
                        rssi = int(stripped.split(": ", 1)[-1])
                    except ValueError:
                        pass
            connected = ssid == WIFI_SSID
            return connected, ssid, rssi
        return False, None, None
    except Exception:
        return False, None, None


def ping_esp32():
    """Ping the ESP32 AP IP and return latency in ms, or None."""
    try:
        result = subprocess.run(
            ["ping", "-c", "1", "-W", "1", ESP32_IP],
            capture_output=True, text=True, timeout=3
        )
        if result.returncode == 0:
            for line in result.stdout.splitlines():
                if "time=" in line:
                    # Extract time=1.234 ms
                    part = line.split("time=")[-1].split()[0]
                    return float(part)
        return None
    except Exception:
        return None


def auto_connect_wifi():
    """Attempt to connect to the 'bridge' WiFi on macOS."""
    if platform.system() != "Darwin":
        print("[WiFi] Auto-connect uniquement supporté sur macOS")
        return False

    connected, ssid, rssi = check_wifi_connection()
    if connected:
        print(f"[WiFi] Déjà connecté à '{WIFI_SSID}' (RSSI: {rssi} dBm)")
        with state.lock:
            state.wifi_connected = True
            state.wifi_rssi = rssi
            state.wifi_ssid_current = ssid
        return True

    print(f"[WiFi] Connexion à '{WIFI_SSID}'...")
    with state.lock:
        state.wifi_reconnecting = True
    try:
        result = subprocess.run(
            ["networksetup", "-setairportnetwork", "en0", WIFI_SSID, WIFI_PASSWORD],
            capture_output=True, text=True, timeout=15
        )
        if result.returncode == 0:
            time.sleep(2)
            connected, ssid, rssi = check_wifi_connection()
            if connected:
                print(f"[WiFi] Connecté à '{WIFI_SSID}' (RSSI: {rssi} dBm)")
                with state.lock:
                    state.wifi_connected = True
                    state.wifi_reconnecting = False
                    state.wifi_rssi = rssi
                    state.wifi_ssid_current = ssid
                return True
        print(f"[WiFi] Échec connexion: {result.stderr.strip()}")
        with state.lock:
            state.wifi_reconnecting = False
        return False
    except Exception as e:
        print(f"[WiFi] Erreur: {e}")
        with state.lock:
            state.wifi_reconnecting = False
        return False


def wifi_watchdog_thread():
    """Background thread monitoring WiFi connection and auto-reconnecting.
    IMPORTANT: Ne force JAMAIS une reconnexion si déjà sur le bon SSID.
    Les reconnexions WiFi coupent le socket UDP et perdent les trames."""
    backoff = WIFI_CHECK_INTERVAL
    last_ping = 0
    consecutive_failures = 0  # Count consecutive check failures before reconnecting

    while True:
        time.sleep(WIFI_CHECK_INTERVAL)

        # 1. Check WiFi link
        connected, ssid, rssi = check_wifi_connection()
        now = time.time()

        with state.lock:
            was_connected = state.wifi_connected
            state.wifi_rssi = rssi
            state.wifi_ssid_current = ssid
            state.wifi_last_check = now

        if connected:
            consecutive_failures = 0
            with state.lock:
                state.wifi_connected = True
                state.wifi_reconnecting = False
            backoff = WIFI_CHECK_INTERVAL  # Reset backoff

            # 2. Periodic ping to ESP32
            if now - last_ping >= PING_INTERVAL:
                latency = ping_esp32()
                last_ping = now
                with state.lock:
                    state.ping_latency_ms = latency
                    state.ping_reachable = latency is not None

        else:
            consecutive_failures += 1

            # Si on est sur un autre réseau (pas bridge), ou pas connecté du tout
            # ET que ça fait plusieurs checks consécutifs → tenter reconnexion
            with state.lock:
                state.wifi_connected = False
                state.ping_reachable = False
                state.ping_latency_ms = None

            if ssid == WIFI_SSID:
                # On est sur bridge mais airport -I retourne un état bizarre
                # Ne PAS reconnecter — ça couperait le WiFi inutilement
                if consecutive_failures <= 3:
                    continue  # Attendre encore

            if consecutive_failures < 3:
                # Attendre 3 échecs consécutifs avant de reconnecter
                # (évite les faux positifs de airport -I)
                continue

            if was_connected:
                print(f"[WiFi] Connexion perdue (ssid={ssid}) ! Tentative de reconnexion...")

            # Auto-reconnect with exponential backoff
            with state.lock:
                state.wifi_reconnecting = True
            ok = auto_connect_wifi()
            if ok:
                consecutive_failures = 0
                with state.lock:
                    state.wifi_reconnect_count += 1
                print(f"[WiFi] Reconnecté (total reconnexions: {state.wifi_reconnect_count})")
                backoff = WIFI_CHECK_INTERVAL
            else:
                backoff = min(backoff * 1.5, RECONNECT_BACKOFF_MAX)
                print(f"[WiFi] Échec reconnexion, prochaine tentative dans {backoff:.0f}s")
                time.sleep(backoff - WIFI_CHECK_INTERVAL)  # Extra wait

        # 3. Update overall connection state
        with state.lock:
            state.update_connection_state()


# =============================================================================
# UDP RECEIVER THREAD
# =============================================================================

def parse_udp_packet(data):
    """Parse an 18-byte UDP CAN packet from ESP32.

    Format:
        0-1:   Magic (0xCA, 0x4E)
        2-5:   CAN ID (uint32 LE)
        6:     DLC
        7:     Flags (bit0=extended)
        8-15:  Data (8 bytes)
        16-17: Sequence (uint16 LE)
    """
    if len(data) != UDP_PACKET_SIZE:
        return None
    if data[0:2] != UDP_MAGIC:
        return None

    can_id = struct.unpack_from('<I', data, 2)[0]
    dlc = data[6]
    is_extended = bool(data[7] & 0x01)
    payload = data[8:16]
    sequence = struct.unpack_from('<H', data, 16)[0]

    return {
        'can_id': can_id,
        'dlc': dlc,
        'is_extended': is_extended,
        'data': payload[:dlc],
        'data_full': payload,
        'sequence': sequence,
        'timestamp': time.time()
    }


def decode_can_message(msg_info):
    """Decode CAN message using DBC database."""
    can_id = msg_info['can_id']
    data = msg_info['data_full']
    ts = msg_info['timestamp']

    if can_id not in msg_by_id:
        return None

    msg_def = msg_by_id[can_id]
    try:
        decoded = msg_def.decode(data, decode_choices=False)
        result = {}
        for sig_name, value in decoded.items():
            # Find signal definition for unit
            unit = ""
            for sig in msg_def.signals:
                if sig.name == sig_name:
                    unit = sig.unit or ""
                    break
            result[sig_name] = {
                'value': value,
                'unit': unit,
                'msg_name': msg_def.name,
                'can_id': can_id
            }
        return result
    except Exception:
        return None


def udp_receiver_thread():
    """Background thread receiving UDP packets from ESP32.
    Handles BATCHED packets: each UDP packet may contain N * 18 bytes
    (multiple CAN frames concatenated by ESP32 for WiFi stability).
    Automatically recreates socket on errors."""

    def create_socket():
        """Create and bind a new UDP socket.
        On macOS, orphan sockets can permanently block a port (no PID to kill).
        SO_REUSEPORT does NOT help in that case.
        Strategy: try primary port, fallback to alternate port."""
        ports_to_try = [UDP_PORT, UDP_PORT + 1, UDP_PORT + 2]

        for port in ports_to_try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            try:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            except (AttributeError, OSError):
                pass
            s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 524288)
            s.settimeout(2.0)
            try:
                s.bind((UDP_BIND_IP, port))
                if port != UDP_PORT:
                    print(f"[UDP] Port {UDP_PORT} bloqué (socket orphelin macOS)")
                    print(f"[UDP] Utilisation du port alternatif {port}")
                    print(f"[UDP] ⚠ Le firmware ESP32 envoie sur {UDP_PORT}.")
                    print(f"[UDP] → Redémarrer le Mac pour libérer le port, "
                          f"ou changer UDP_PORT dans le firmware.")
                return s
            except OSError:
                s.close()
                continue

        # All ports failed — raise error for retry logic
        raise OSError(f"Cannot bind to ports {ports_to_try}")

    def process_frame(msg_info):
        """Process a single parsed CAN frame into state."""
        with state.lock:
            state.rx_count += 1
            state.record_packet_time(msg_info['timestamp'])

            # Check sequence
            expected = (state.last_sequence + 1) & 0xFFFF
            if state.last_sequence >= 0 and msg_info['sequence'] != expected:
                state.sequence_errors += 1
            state.last_sequence = msg_info['sequence']

            # Update connection state
            state.update_connection_state()

            # Store raw message
            state.messages.append(msg_info)

            # Decode signals
            decoded = decode_can_message(msg_info)
            if decoded:
                ts = msg_info['timestamp']
                for sig_name, sig_info in decoded.items():
                    val = sig_info['value']
                    if isinstance(val, (int, float)):
                        state.signals[sig_name].append((ts, val))
                        state.last_values[sig_name] = (
                            ts, val, sig_info['unit'], sig_info['msg_name']
                        )
            else:
                state.decode_errors += 1

            # BLF recording
            if state.recording and state.blf_writer:
                try:
                    can_msg = can.Message(
                        arbitration_id=msg_info['can_id'],
                        data=msg_info['data_full'][:msg_info['dlc']],
                        is_extended_id=msg_info['is_extended'],
                        timestamp=msg_info['timestamp']
                    )
                    state.blf_writer(can_msg)
                    state.record_count += 1
                except Exception:
                    pass

    FRAME_SIZE = 18  # bytes per CAN frame in UDP packet
    sock = None
    consecutive_errors = 0
    MAX_ERRORS_BEFORE_REBIND = 10

    # Register cleanup to prevent orphan sockets on macOS
    def cleanup_socket():
        nonlocal sock
        if sock is not None:
            try:
                sock.close()
                print("[UDP] Socket fermé proprement")
            except Exception:
                pass
            sock = None
    atexit.register(cleanup_socket)
    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            prev = signal.getsignal(sig)
            def handler(s, f, prev=prev):
                cleanup_socket()
                if callable(prev) and prev not in (signal.SIG_DFL, signal.SIG_IGN):
                    prev(s, f)
            signal.signal(sig, handler)
        except (OSError, ValueError):
            pass  # Can't set signal handler in non-main thread

    while True:
        # Create socket if needed
        if sock is None:
            try:
                sock = create_socket()
                bound_port = sock.getsockname()[1]
                print(f"[UDP] Écoute sur {UDP_BIND_IP}:{bound_port} (mode batch)")
                with state.lock:
                    state.udp_active = True
                consecutive_errors = 0
            except Exception as e:
                print(f"[UDP] Erreur bind: {e} — retry dans 3s")
                with state.lock:
                    state.udp_active = False
                time.sleep(3)
                continue

        try:
            # Receive up to 2KB (enough for ~110 frames per batch)
            data, addr = sock.recvfrom(2048)
            consecutive_errors = 0

            data_len = len(data)

            # Handle batched packets: extract each 18-byte frame
            if data_len >= FRAME_SIZE and data_len % FRAME_SIZE == 0:
                # Batched packet: N concatenated frames
                offset = 0
                while offset + FRAME_SIZE <= data_len:
                    frame_data = data[offset:offset + FRAME_SIZE]
                    msg_info = parse_udp_packet(frame_data)
                    if msg_info is not None:
                        process_frame(msg_info)
                    offset += FRAME_SIZE
            elif data_len == FRAME_SIZE:
                # Single frame (legacy)
                msg_info = parse_udp_packet(data)
                if msg_info is not None:
                    process_frame(msg_info)
            else:
                # Unexpected size — try to parse what we can
                offset = 0
                while offset + FRAME_SIZE <= data_len:
                    frame_data = data[offset:offset + FRAME_SIZE]
                    msg_info = parse_udp_packet(frame_data)
                    if msg_info is not None:
                        process_frame(msg_info)
                    offset += FRAME_SIZE

        except socket.timeout:
            continue
        except OSError as e:
            consecutive_errors += 1
            print(f"[UDP] Erreur socket ({consecutive_errors}): {e}")
            if consecutive_errors >= MAX_ERRORS_BEFORE_REBIND:
                print("[UDP] Trop d'erreurs — recréation du socket...")
                try:
                    sock.close()
                except Exception:
                    pass
                sock = None
                time.sleep(1)
        except Exception as e:
            print(f"[UDP] Erreur: {e}")
            time.sleep(0.1)


# =============================================================================
# BLF RECORDING
# =============================================================================

def start_recording():
    """Start recording CAN messages to BLF file."""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    recordings_dir = SCRIPT_DIR / "recordings"
    recordings_dir.mkdir(exist_ok=True)
    blf_path = recordings_dir / f"tesla_can_{timestamp}.blf"

    try:
        writer = can.BLFWriter(str(blf_path))
        with state.lock:
            state.blf_writer = writer
            state.blf_path = str(blf_path)
            state.record_count = 0
            state.recording = True
        print(f"[REC] Enregistrement démarré: {blf_path.name}")
        return True
    except Exception as e:
        print(f"[REC] Erreur: {e}")
        return False


def stop_recording():
    """Stop BLF recording."""
    with state.lock:
        state.recording = False
        if state.blf_writer:
            try:
                state.blf_writer.stop()
            except Exception:
                pass
            state.blf_writer = None
        path = state.blf_path
        count = state.record_count
    print(f"[REC] Enregistrement arrêté: {count} trames")
    return path, count


# =============================================================================
# BLF REPLAY
# =============================================================================

class ReplayState:
    """State for BLF file replay."""
    def __init__(self):
        self.lock = threading.Lock()
        self.active = False
        self.paused = False
        self.file_path = None
        self.file_name = None
        self.speed = 1.0          # Playback speed multiplier
        self.progress = 0.0       # 0.0 to 1.0
        self.total_frames = 0
        self.played_frames = 0
        self.duration_s = 0.0     # Total duration of the recording
        self.elapsed_s = 0.0      # Current playback position
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()   # Not paused initially
        self._thread = None


replay_state = ReplayState()


def list_recordings():
    """List available BLF recordings as dropdown options."""
    recordings_dir = SCRIPT_DIR / "recordings"
    if not recordings_dir.exists():
        return []
    files = sorted(recordings_dir.glob("*.blf"), reverse=True)
    options = []
    for f in files:
        size_kb = f.stat().st_size / 1024
        label = f"{f.name}  ({size_kb:.0f} KB)"
        options.append({"label": label, "value": str(f)})
    return options


def blf_replay_thread(file_path, speed=1.0):
    """Background thread that reads a BLF file and injects frames into state."""
    print(f"[REPLAY] Démarrage: {os.path.basename(file_path)} (x{speed})")

    try:
        reader = can.BLFReader(file_path)
        messages = list(reader)
    except Exception as e:
        print(f"[REPLAY] Erreur lecture BLF: {e}")
        with replay_state.lock:
            replay_state.active = False
        return

    if not messages:
        print("[REPLAY] Fichier vide")
        with replay_state.lock:
            replay_state.active = False
        return

    total = len(messages)
    t_start_file = messages[0].timestamp
    t_end_file = messages[-1].timestamp
    duration = t_end_file - t_start_file if t_end_file > t_start_file else 1.0

    with replay_state.lock:
        replay_state.total_frames = total
        replay_state.played_frames = 0
        replay_state.duration_s = duration
        replay_state.elapsed_s = 0.0
        replay_state.progress = 0.0

    print(f"[REPLAY] {total} trames, durée: {duration:.1f}s")

    # Reset dashboard state for replay
    with state.lock:
        state.messages.clear()
        state.signals.clear()
        state.last_values.clear()
        state.rx_count = 0
        state.decode_errors = 0
        state.sequence_errors = 0
        state.last_sequence = -1
        state.start_time = time.time()
        state.connection_state = 'receiving'
        state.packet_rate = 0
        state._rate_window.clear()

    prev_file_ts = t_start_file
    seq = 0

    for i, msg in enumerate(messages):
        # Check stop
        if replay_state._stop_event.is_set():
            break

        # Check pause (blocks until unpaused)
        replay_state._pause_event.wait()

        # Re-check stop after unpause
        if replay_state._stop_event.is_set():
            break

        # Timing: wait proportional to original inter-frame delay
        file_delta = msg.timestamp - prev_file_ts
        if file_delta > 0 and speed > 0:
            real_delay = file_delta / speed
            if real_delay > 0.0001:  # Skip sub-0.1ms delays
                # Use small sleep increments to stay responsive to stop/pause
                deadline = time.time() + real_delay
                while time.time() < deadline:
                    if replay_state._stop_event.is_set():
                        break
                    remaining = deadline - time.time()
                    time.sleep(min(remaining, 0.01))
        prev_file_ts = msg.timestamp

        if replay_state._stop_event.is_set():
            break

        # Inject frame into dashboard state
        now = time.time()
        raw_data = bytes(msg.data)
        data_full = (raw_data + b'\x00' * 8)[:8]

        msg_info = {
            'can_id': msg.arbitration_id,
            'dlc': msg.dlc if hasattr(msg, 'dlc') else len(msg.data),
            'is_extended': msg.is_extended_id,
            'data': raw_data[:msg.dlc] if hasattr(msg, 'dlc') else raw_data,
            'data_full': data_full,
            'sequence': seq & 0xFFFF,
            'timestamp': now,
        }
        seq += 1

        with state.lock:
            state.rx_count += 1
            state.record_packet_time(now)
            state.last_sequence = msg_info['sequence']
            state.connection_state = 'receiving'
            state.messages.append(msg_info)

            decoded = decode_can_message(msg_info)
            if decoded:
                for sig_name, sig_info in decoded.items():
                    val = sig_info['value']
                    if isinstance(val, (int, float)):
                        state.signals[sig_name].append((now, val))
                        state.last_values[sig_name] = (
                            now, val, sig_info['unit'], sig_info['msg_name']
                        )
            else:
                state.decode_errors += 1

        with replay_state.lock:
            replay_state.played_frames = i + 1
            elapsed = msg.timestamp - t_start_file
            replay_state.elapsed_s = elapsed
            replay_state.progress = (i + 1) / total

    # Done
    with replay_state.lock:
        replay_state.active = False
        replay_state.paused = False
        if replay_state.progress >= 0.99:
            replay_state.progress = 1.0

    print(f"[REPLAY] Terminé: {replay_state.played_frames}/{total} trames")


def start_replay(file_path, speed=1.0):
    """Start replaying a BLF file."""
    stop_replay()  # Stop any current replay
    time.sleep(0.1)
    with replay_state.lock:
        replay_state.active = True
        replay_state.paused = False
        replay_state.file_path = file_path
        replay_state.file_name = os.path.basename(file_path)
        replay_state.speed = speed
        replay_state.progress = 0.0
        replay_state.played_frames = 0
        replay_state._stop_event.clear()
        replay_state._pause_event.set()
        replay_state._thread = threading.Thread(
            target=blf_replay_thread, args=(file_path, speed), daemon=True
        )
        replay_state._thread.start()


def stop_replay():
    """Stop current replay."""
    with replay_state.lock:
        replay_state._stop_event.set()
        replay_state._pause_event.set()  # Unpause so thread can exit
    if replay_state._thread and replay_state._thread.is_alive():
        replay_state._thread.join(timeout=3)
    with replay_state.lock:
        replay_state.active = False
        replay_state.paused = False


def pause_replay():
    """Toggle pause on current replay."""
    with replay_state.lock:
        if replay_state.paused:
            replay_state.paused = False
            replay_state._pause_event.set()
            print("[REPLAY] Reprise")
        else:
            replay_state.paused = True
            replay_state._pause_event.clear()
            print("[REPLAY] Pause")


# =============================================================================
# DASH APPLICATION
# =============================================================================

app = dash.Dash(
    __name__,
    external_stylesheets=[dbc.themes.DARKLY],
    assets_folder=os.path.join(os.path.dirname(os.path.abspath(__file__)), "assets"),
    title="Tesla CAN Dashboard",
    update_title=None,
)

# ─── Key signals for graphs ─────────────────────────────────────────────────
# These are the main signal categories for the dashboard graphs
GRAPH_SIGNALS = {
    "Vitesse & Conduite": {
        "signals": ["DI_vehicleSpeed", "DI_gear", "ESP_vehicleSpeed",
                     "UI_speedLimit", "GPS_vehicleSpeed", "DI_uiSpeed"],
        "unit": "kph",
        "y_label": "Vitesse (kph)"
    },
    "Batterie": {
        "signals": ["BattVoltage132", "SmoothBattCurrent132", "SOCUI292",
                     "BMS_nominalEnergyRemaining", "BMS_nominalFullPackEnergy"],
        "unit": "mixed",
        "y_label": "Valeur"
    },
    "Puissance Moteur": {
        "signals": ["RearPower266", "FrontPower2E5", "DIR_torqueActual",
                     "DIF_torqueActual", "RearTorque1D8", "FrontTorque1D5"],
        "unit": "mixed",
        "y_label": "Puissance (kW) / Couple (Nm)"
    },
    "Températures": {
        "signals": ["RearTempStator315", "RearTempInverter315",
                     "RearTempInvHeatsink315", "TempStator376",
                     "TempInverter376", "TempInvHeatsink376",
                     "BMSmaxPackTemperature", "BMSminPackTemperature"],
        "unit": "°C",
        "y_label": "Température (°C)"
    },
    "Freinage & ESP": {
        "signals": ["ESP_brakeTorqueTarget", "ESP_brakeApply",
                     "ESP_driverBrakeApply", "IBST_driverBrakeApply"],
        "unit": "mixed",
        "y_label": "Valeur"
    },
}


# ─── Layout ──────────────────────────────────────────────────────────────────

def make_status_badge(text, color="secondary"):
    return dbc.Badge(text, color=color, className="me-2 p-2")


app.layout = dbc.Container([
    # ── Header ──
    dbc.Row([
        dbc.Col([
            html.H2("Tesla CAN Dashboard", className="text-light mb-0"),
            html.Small("Récepteur CAN en temps réel via ESP32 WiFi Bridge",
                       className="text-muted"),
        ], width=6),
        dbc.Col([
            html.Div(id="status-badges", className="text-end mt-2"),
        ], width=6),
    ], className="my-3 border-bottom border-secondary pb-2"),

    # ── Connection Status Panel ──
    dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardBody([
                    dbc.Row([
                        # Connection state indicator
                        dbc.Col([
                            html.Div(id="conn-indicator",
                                     style={"width": "14px", "height": "14px",
                                            "borderRadius": "50%",
                                            "display": "inline-block",
                                            "verticalAlign": "middle",
                                            "marginRight": "8px"}),
                            html.Span(id="conn-state-text",
                                      className="fw-bold",
                                      style={"verticalAlign": "middle"}),
                        ], width=3, className="d-flex align-items-center"),
                        # WiFi details
                        dbc.Col([
                            html.Span("WiFi: ", className="text-muted small"),
                            html.Span(id="conn-wifi-ssid", className="small"),
                            html.Span(" | RSSI: ", className="text-muted small"),
                            html.Span(id="conn-wifi-rssi", className="small"),
                        ], width=3, className="d-flex align-items-center"),
                        # Ping / latency
                        dbc.Col([
                            html.Span("Ping ESP32: ", className="text-muted small"),
                            html.Span(id="conn-ping", className="small"),
                            html.Span(" | Débit: ", className="text-muted small"),
                            html.Span(id="conn-rate", className="small"),
                        ], width=3, className="d-flex align-items-center"),
                        # Reconnections & last packet
                        dbc.Col([
                            html.Span(id="conn-last-pkt", className="small text-muted"),
                            html.Span(" | ", className="text-muted small"),
                            html.Span(id="conn-reconnects", className="small text-muted"),
                        ], width=3, className="d-flex align-items-center"),
                    ]),
                ], className="py-2"),
            ], color="dark", outline=True, className="mb-2"),
        ]),
    ]),

    # ── Controls: Recording + Replay ──
    dbc.Row([
        dbc.Col([
            dbc.ButtonGroup([
                dbc.Button(
                    [html.I(className="bi bi-record-circle me-1"), "Enregistrer"],
                    id="btn-record", color="danger", outline=True, size="sm"
                ),
                dbc.Button(
                    [html.I(className="bi bi-stop-circle me-1"), "Arrêter"],
                    id="btn-stop", color="warning", outline=True, size="sm",
                    disabled=True
                ),
            ]),
            html.Span(id="record-status", className="ms-3 text-muted small"),
        ], width=4),
        dbc.Col([
            html.Div(id="stats-bar", className="text-end text-muted small"),
        ], width=8),
    ], className="mb-2"),

    # ── Replay Panel ──
    dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardBody([
                    dbc.Row([
                        # File selector
                        dbc.Col([
                            html.Span("Rejouer: ",
                                      className="small fw-bold text-muted me-2",
                                      style={"whiteSpace": "nowrap"}),
                            dbc.Select(
                                id="replay-file-select",
                                options=list_recordings(),
                                placeholder="Sélectionner un fichier BLF...",
                                style={"fontSize": "12px",
                                       "minWidth": "280px",
                                       "backgroundColor": "#222",
                                       "color": "#ddd",
                                       "borderColor": "#555"},
                                className="d-inline-block flex-grow-1",
                            ),
                        ], width=5, className="d-flex align-items-center"),
                        # Speed selector
                        dbc.Col([
                            html.Span("Vitesse: ",
                                      className="small text-muted me-1"),
                            dbc.Select(
                                id="replay-speed",
                                options=[
                                    {"label": "x0.25", "value": "0.25"},
                                    {"label": "x0.5", "value": "0.5"},
                                    {"label": "x1", "value": "1"},
                                    {"label": "x2", "value": "2"},
                                    {"label": "x5", "value": "5"},
                                    {"label": "x10", "value": "10"},
                                    {"label": "Max", "value": "100"},
                                ],
                                value="1",
                                size="sm",
                                style={"width": "80px", "fontSize": "12px",
                                       "backgroundColor": "#333",
                                       "color": "#ddd",
                                       "display": "inline-block"},
                            ),
                        ], width=2, className="d-flex align-items-center"),
                        # Play / Pause / Stop buttons
                        dbc.Col([
                            dbc.ButtonGroup([
                                dbc.Button("Lire",
                                           id="btn-replay-play",
                                           color="success",
                                           outline=True, size="sm"),
                                dbc.Button("Pause",
                                           id="btn-replay-pause",
                                           color="info",
                                           outline=True, size="sm",
                                           disabled=True),
                                dbc.Button("Stop",
                                           id="btn-replay-stop",
                                           color="danger",
                                           outline=True, size="sm",
                                           disabled=True),
                                dbc.Button("Rafraîchir",
                                           id="btn-replay-refresh",
                                           color="secondary",
                                           outline=True, size="sm"),
                            ]),
                        ], width=3, className="d-flex align-items-center"),
                        # Status
                        dbc.Col([
                            html.Span(id="replay-status",
                                      className="small text-muted"),
                        ], width=2,
                           className="d-flex align-items-center"),
                    ]),
                    # Progress bar
                    dbc.Row([
                        dbc.Col([
                            dbc.Progress(
                                id="replay-progress",
                                value=0, max=100,
                                style={"height": "6px",
                                       "marginTop": "6px"},
                                color="success",
                                className="bg-dark",
                            ),
                        ]),
                    ], className="mt-1"),
                ], className="py-2"),
            ], color="dark", outline=True, className="mb-2"),
        ]),
    ]),

    # ══════════════════════════════════════════════════════════════════════════
    # ── VIRTUAL COCKPIT ──────────────────────────────────────────────────────
    # ══════════════════════════════════════════════════════════════════════════
    dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardHeader([
                    html.Span("Cockpit Virtuel Tesla", className="fw-bold"),
                ], className="py-2 text-center",
                   style={"borderBottom": "2px solid #444"}),
                dbc.CardBody([

                    # ── Row 1: Speed center + Gear + Indicators ──
                    dbc.Row([
                        # Left: Turn signals + Blind spot L
                        dbc.Col([
                            html.Div([
                                html.Div(id="ck-turn-left",
                                         children="◀",
                                         style={"fontSize": "36px",
                                                "textAlign": "center",
                                                "color": "#333"}),
                                html.Div("Angle mort G",
                                         className="text-muted",
                                         style={"fontSize": "9px",
                                                "textAlign": "center"}),
                                html.Div(id="ck-blind-left",
                                         children="--",
                                         style={"fontSize": "14px",
                                                "textAlign": "center",
                                                "color": "#555"}),
                            ]),
                        ], width=2, className="d-flex align-items-center "
                           "justify-content-center"),

                        # Center: Speed + Gear
                        dbc.Col([
                            html.Div([
                                # Gear indicator
                                html.Div(id="ck-gear",
                                         children="P",
                                         style={"fontSize": "20px",
                                                "fontWeight": "bold",
                                                "textAlign": "center",
                                                "color": "#888",
                                                "marginBottom": "2px"}),
                                # Main speed
                                html.Div(id="ck-speed",
                                         children="---",
                                         style={"fontSize": "64px",
                                                "fontWeight": "bold",
                                                "textAlign": "center",
                                                "color": "#fff",
                                                "lineHeight": "1",
                                                "fontFamily":
                                                    "monospace"}),
                                html.Div(id="ck-speed-unit",
                                         children="km/h",
                                         style={"fontSize": "12px",
                                                "textAlign": "center",
                                                "color": "#888",
                                                "marginTop": "-2px"}),
                                # Speed limit
                                html.Div([
                                    html.Span("Limite: ",
                                              className="text-muted",
                                              style={"fontSize": "10px"}),
                                    html.Span(id="ck-speed-limit",
                                              children="--",
                                              style={"fontSize": "13px",
                                                     "color": "#ff6666",
                                                     "fontWeight":
                                                         "bold"}),
                                ], style={"textAlign": "center",
                                          "marginTop": "2px"}),
                            ]),
                        ], width=3, className="d-flex align-items-center "
                           "justify-content-center"),

                        # Right: Turn signals + Blind spot R
                        dbc.Col([
                            html.Div([
                                html.Div(id="ck-turn-right",
                                         children="▶",
                                         style={"fontSize": "36px",
                                                "textAlign": "center",
                                                "color": "#333"}),
                                html.Div("Angle mort D",
                                         className="text-muted",
                                         style={"fontSize": "9px",
                                                "textAlign": "center"}),
                                html.Div(id="ck-blind-right",
                                         children="--",
                                         style={"fontSize": "14px",
                                                "textAlign": "center",
                                                "color": "#555"}),
                            ]),
                        ], width=2, className="d-flex align-items-center "
                           "justify-content-center"),

                        # Battery gauge
                        dbc.Col([
                            html.Div([
                                html.Div([
                                    html.Span("🔋",
                                              style={"fontSize": "16px"}),
                                    html.Span(" SOC",
                                              className="text-muted",
                                              style={"fontSize": "10px"}),
                                ], style={"textAlign": "center"}),
                                html.Div(id="ck-soc",
                                         children="--%",
                                         style={"fontSize": "28px",
                                                "fontWeight": "bold",
                                                "textAlign": "center",
                                                "color": "#00cc66"}),
                                dbc.Progress(
                                    id="ck-soc-bar",
                                    value=0, max=100,
                                    style={"height": "8px",
                                           "marginTop": "2px"},
                                    color="success",
                                    className="bg-dark",
                                ),
                                html.Div(id="ck-range",
                                         children="-- km",
                                         style={"fontSize": "11px",
                                                "textAlign": "center",
                                                "color": "#aaa",
                                                "marginTop": "2px"}),
                            ]),
                        ], width=2),

                        # Power / Torque
                        dbc.Col([
                            html.Div([
                                html.Div("⚡ Puissance",
                                         className="text-muted",
                                         style={"fontSize": "10px",
                                                "textAlign": "center"}),
                                html.Div(id="ck-power",
                                         children="-- kW",
                                         style={"fontSize": "22px",
                                                "fontWeight": "bold",
                                                "textAlign": "center",
                                                "color": "#4dc9f6"}),
                                html.Div("Couple",
                                         className="text-muted",
                                         style={"fontSize": "10px",
                                                "textAlign": "center"}),
                                html.Div(id="ck-torque",
                                         children="-- Nm",
                                         style={"fontSize": "15px",
                                                "textAlign": "center",
                                                "color": "#aaa"}),
                            ]),
                        ], width=2),
                    ], className="mb-3"),

                    # ── Row 2: Autopilot + Lights + Doors + HVAC ──
                    dbc.Row([
                        # Autopilot status
                        dbc.Col([
                            dbc.Card([
                                dbc.CardBody([
                                    html.Div("🤖 Autopilot",
                                             className="text-muted",
                                             style={"fontSize": "10px",
                                                    "textAlign": "center"}),
                                    html.Div(id="ck-autopilot",
                                             children="OFF",
                                             style={"fontSize": "16px",
                                                    "fontWeight": "bold",
                                                    "textAlign": "center",
                                                    "color": "#555"}),
                                    html.Div(id="ck-cruise-speed",
                                             children="",
                                             style={"fontSize": "11px",
                                                    "textAlign": "center",
                                                    "color": "#888"}),
                                    html.Div([
                                        html.Span(id="ck-lane-depart",
                                                  children="LDW",
                                                  style={"fontSize": "10px",
                                                         "color": "#444",
                                                         "marginRight":
                                                             "6px"}),
                                        html.Span(id="ck-fcw",
                                                  children="FCW",
                                                  style={"fontSize": "10px",
                                                         "color": "#444",
                                                         "marginRight":
                                                             "6px"}),
                                        html.Span(id="ck-aeb",
                                                  children="AEB",
                                                  style={"fontSize": "10px",
                                                         "color": "#444"}),
                                    ], style={"textAlign": "center",
                                              "marginTop": "4px"}),
                                ], className="p-2"),
                            ], color="dark", outline=True,
                               style={"minHeight": "100px"}),
                        ], width=3),

                        # Lights
                        dbc.Col([
                            dbc.Card([
                                dbc.CardBody([
                                    html.Div("💡 Éclairage",
                                             className="text-muted",
                                             style={"fontSize": "10px",
                                                    "textAlign": "center"}),
                                    html.Div([
                                        html.Span(id="ck-headlights",
                                                  children="🔦",
                                                  title="Phares",
                                                  style={"fontSize": "18px",
                                                         "marginRight":
                                                             "8px",
                                                         "opacity":
                                                             "0.3"}),
                                        html.Span(id="ck-highbeam",
                                                  children="🔆",
                                                  title="Pleins phares",
                                                  style={"fontSize": "18px",
                                                         "marginRight":
                                                             "8px",
                                                         "opacity":
                                                             "0.3"}),
                                        html.Span(id="ck-fog",
                                                  children="🌫️",
                                                  title="Antibrouillard",
                                                  style={"fontSize": "18px",
                                                         "marginRight":
                                                             "8px",
                                                         "opacity":
                                                             "0.3"}),
                                        html.Span(id="ck-hazard",
                                                  children="⚠️",
                                                  title="Warnings",
                                                  style={"fontSize": "18px",
                                                         "opacity":
                                                             "0.3"}),
                                    ], style={"textAlign": "center",
                                              "marginTop": "4px"}),
                                    html.Div([
                                        html.Span(id="ck-brake-light",
                                                  children="BRAKE",
                                                  style={"fontSize": "10px",
                                                         "color": "#444",
                                                         "marginRight":
                                                             "8px"}),
                                        html.Span(id="ck-regen-light",
                                                  children="REGEN",
                                                  style={"fontSize": "10px",
                                                         "color": "#444"}),
                                    ], style={"textAlign": "center",
                                              "marginTop": "4px"}),
                                ], className="p-2"),
                            ], color="dark", outline=True,
                               style={"minHeight": "100px"}),
                        ], width=3),

                        # Doors / Trunk
                        dbc.Col([
                            dbc.Card([
                                dbc.CardBody([
                                    html.Div("🚗 Portes",
                                             className="text-muted",
                                             style={"fontSize": "10px",
                                                    "textAlign": "center"}),
                                    html.Div([
                                        html.Span(id="ck-door-fl",
                                                  children="FL",
                                                  style={"fontSize": "11px",
                                                         "color": "#444",
                                                         "margin": "0 4px",
                                                         "padding": "2px "
                                                         "4px",
                                                         "borderRadius":
                                                             "3px",
                                                         "border":
                                                             "1px solid "
                                                             "#333"}),
                                        html.Span(id="ck-door-fr",
                                                  children="FR",
                                                  style={"fontSize": "11px",
                                                         "color": "#444",
                                                         "margin": "0 4px",
                                                         "padding": "2px "
                                                         "4px",
                                                         "borderRadius":
                                                             "3px",
                                                         "border":
                                                             "1px solid "
                                                             "#333"}),
                                        html.Span(id="ck-door-rl",
                                                  children="RL",
                                                  style={"fontSize": "11px",
                                                         "color": "#444",
                                                         "margin": "0 4px",
                                                         "padding": "2px "
                                                         "4px",
                                                         "borderRadius":
                                                             "3px",
                                                         "border":
                                                             "1px solid "
                                                             "#333"}),
                                        html.Span(id="ck-door-rr",
                                                  children="RR",
                                                  style={"fontSize": "11px",
                                                         "color": "#444",
                                                         "margin": "0 4px",
                                                         "padding": "2px "
                                                         "4px",
                                                         "borderRadius":
                                                             "3px",
                                                         "border":
                                                             "1px solid "
                                                             "#333"}),
                                    ], style={"textAlign": "center",
                                              "marginTop": "4px"}),
                                    html.Div([
                                        html.Span(id="ck-trunk",
                                                  children="Coffre",
                                                  style={"fontSize": "10px",
                                                         "color": "#444",
                                                         "marginRight":
                                                             "8px"}),
                                        html.Span(id="ck-frunk",
                                                  children="Frunk",
                                                  style={"fontSize": "10px",
                                                         "color": "#444"}),
                                    ], style={"textAlign": "center",
                                              "marginTop": "4px"}),
                                    html.Div(id="ck-charge-port",
                                             children="Port: --",
                                             style={"fontSize": "10px",
                                                    "textAlign": "center",
                                                    "color": "#444",
                                                    "marginTop": "2px"}),
                                ], className="p-2"),
                            ], color="dark", outline=True,
                               style={"minHeight": "100px"}),
                        ], width=3),

                        # Battery / Charge details
                        dbc.Col([
                            dbc.Card([
                                dbc.CardBody([
                                    html.Div("🔌 Batterie HV",
                                             className="text-muted",
                                             style={"fontSize": "10px",
                                                    "textAlign": "center"}),
                                    html.Div([
                                        html.Span(id="ck-pack-v",
                                                  children="-- V",
                                                  style={"fontSize": "14px",
                                                         "color": "#4dc9f6",
                                                         "fontWeight":
                                                             "bold"}),
                                        html.Span(" / ",
                                                  style={"color": "#555"}),
                                        html.Span(id="ck-pack-a",
                                                  children="-- A",
                                                  style={"fontSize": "14px",
                                                         "color": "#f6c14d"}),
                                    ], style={"textAlign": "center",
                                              "marginTop": "4px"}),
                                    html.Div(id="ck-charge-status",
                                             children="--",
                                             style={"fontSize": "11px",
                                                    "textAlign": "center",
                                                    "color": "#888",
                                                    "marginTop": "4px"}),
                                    html.Div(id="ck-12v",
                                             children="12V: --",
                                             style={"fontSize": "10px",
                                                    "textAlign": "center",
                                                    "color": "#666",
                                                    "marginTop": "2px"}),
                                ], className="p-2"),
                            ], color="dark", outline=True,
                               style={"minHeight": "100px"}),
                        ], width=3),
                    ], className="mb-3"),

                    # ── Row 3: Brake / Pedal / Steering / Tire / Temps ──
                    dbc.Row([
                        # Pedals
                        dbc.Col([
                            html.Div([
                                html.Div("Pédale",
                                         className="text-muted",
                                         style={"fontSize": "10px",
                                                "textAlign": "center"}),
                                html.Div(id="ck-accel-pedal",
                                         children="0%",
                                         style={"fontSize": "14px",
                                                "textAlign": "center",
                                                "color": "#00cc66"}),
                                html.Div(id="ck-brake-pedal",
                                         children="Frein: --",
                                         style={"fontSize": "11px",
                                                "textAlign": "center",
                                                "color": "#888"}),
                            ]),
                        ], width=2),

                        # Steering
                        dbc.Col([
                            html.Div([
                                html.Div("Volant",
                                         className="text-muted",
                                         style={"fontSize": "10px",
                                                "textAlign": "center"}),
                                html.Div(id="ck-steer-angle",
                                         children="0.0°",
                                         style={"fontSize": "18px",
                                                "fontWeight": "bold",
                                                "textAlign": "center",
                                                "color": "#fff"}),
                            ]),
                        ], width=2),

                        # Wheel speeds
                        dbc.Col([
                            html.Div([
                                html.Div("Roues (km/h)",
                                         className="text-muted",
                                         style={"fontSize": "10px",
                                                "textAlign": "center"}),
                                html.Div([
                                    html.Span(id="ck-whl-fl",
                                              children="--",
                                              style={"fontSize": "11px",
                                                     "color": "#aaa",
                                                     "margin": "0 3px"}),
                                    html.Span(id="ck-whl-fr",
                                              children="--",
                                              style={"fontSize": "11px",
                                                     "color": "#aaa",
                                                     "margin": "0 3px"}),
                                ], style={"textAlign": "center"}),
                                html.Div([
                                    html.Span(id="ck-whl-rl",
                                              children="--",
                                              style={"fontSize": "11px",
                                                     "color": "#aaa",
                                                     "margin": "0 3px"}),
                                    html.Span(id="ck-whl-rr",
                                              children="--",
                                              style={"fontSize": "11px",
                                                     "color": "#aaa",
                                                     "margin": "0 3px"}),
                                ], style={"textAlign": "center"}),
                            ]),
                        ], width=2),

                        # TPMS
                        dbc.Col([
                            html.Div([
                                html.Div("Pneus (bar)",
                                         className="text-muted",
                                         style={"fontSize": "10px",
                                                "textAlign": "center"}),
                                html.Div([
                                    html.Span(id="ck-tpms-fl",
                                              children="--",
                                              style={"fontSize": "11px",
                                                     "color": "#aaa",
                                                     "margin": "0 3px"}),
                                    html.Span(id="ck-tpms-fr",
                                              children="--",
                                              style={"fontSize": "11px",
                                                     "color": "#aaa",
                                                     "margin": "0 3px"}),
                                ], style={"textAlign": "center"}),
                                html.Div([
                                    html.Span(id="ck-tpms-rl",
                                              children="--",
                                              style={"fontSize": "11px",
                                                     "color": "#aaa",
                                                     "margin": "0 3px"}),
                                    html.Span(id="ck-tpms-rr",
                                              children="--",
                                              style={"fontSize": "11px",
                                                     "color": "#aaa",
                                                     "margin": "0 3px"}),
                                ], style={"textAlign": "center"}),
                            ]),
                        ], width=2),

                        # Temperatures
                        dbc.Col([
                            html.Div([
                                html.Div("🌡️ Températures",
                                         className="text-muted",
                                         style={"fontSize": "10px",
                                                "textAlign": "center"}),
                                html.Div(id="ck-temp-battery",
                                         children="Batt: --°C",
                                         style={"fontSize": "11px",
                                                "textAlign": "center",
                                                "color": "#aaa"}),
                                html.Div(id="ck-temp-cabin",
                                         children="Hab: --°C",
                                         style={"fontSize": "11px",
                                                "textAlign": "center",
                                                "color": "#aaa"}),
                                html.Div(id="ck-temp-exterior",
                                         children="Ext: --°C",
                                         style={"fontSize": "11px",
                                                "textAlign": "center",
                                                "color": "#aaa"}),
                            ]),
                        ], width=2),

                        # Odometer + GPS
                        dbc.Col([
                            html.Div([
                                html.Div("📍 GPS / Odo",
                                         className="text-muted",
                                         style={"fontSize": "10px",
                                                "textAlign": "center"}),
                                html.Div(id="ck-odometer",
                                         children="-- km",
                                         style={"fontSize": "12px",
                                                "textAlign": "center",
                                                "color": "#aaa"}),
                                html.Div(id="ck-gps-speed",
                                         children="GPS: -- km/h",
                                         style={"fontSize": "11px",
                                                "textAlign": "center",
                                                "color": "#888"}),
                            ]),
                        ], width=2),
                    ]),

                ], className="p-3",
                   style={"backgroundColor": "#111",
                          "borderRadius": "8px"}),
            ], color="dark", outline=True, className="mb-3"),
        ]),
    ]),

    # ── Graphs ──
    dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardHeader(title, className="py-1 small fw-bold"),
                dbc.CardBody([
                    dcc.Graph(
                        id=f"graph-{i}",
                        config={"displayModeBar": False},
                        style={"height": "200px"},
                    )
                ], className="p-1"),
            ], className="mb-2", color="dark", outline=True)
        ], width=6)
        for i, title in enumerate(GRAPH_SIGNALS.keys())
    ], className="mb-3"),

    # ── Signal Table ──
    dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardHeader([
                    html.Span("Signaux Décodés", className="fw-bold"),
                    html.Span(id="signal-count",
                              className="ms-2 badge bg-info"),
                ], className="py-2"),
                dbc.CardBody([
                    dash_table.DataTable(
                        id="signal-table",
                        columns=[
                            {"name": "Signal", "id": "signal"},
                            {"name": "Valeur", "id": "value"},
                            {"name": "Unité", "id": "unit"},
                            {"name": "Message CAN", "id": "msg_name"},
                            {"name": "CAN ID", "id": "can_id"},
                        ],
                        data=[],
                        style_table={"overflowY": "auto",
                                     "minHeight": "420px",
                                     "maxHeight": "420px"},
                        style_header={
                            "backgroundColor": "#303030",
                            "color": "#fff",
                            "fontWeight": "bold",
                            "fontSize": "12px",
                            "position": "sticky",
                            "top": 0,
                            "zIndex": 1,
                        },
                        style_filter={
                            "backgroundColor": "#1a1a1a",
                            "color": "#eee",
                            "fontSize": "11px",
                            "padding": "2px 4px",
                            "border": "1px solid #555",
                        },
                        style_cell={
                            "backgroundColor": "#222",
                            "color": "#ddd",
                            "fontSize": "11px",
                            "padding": "4px 8px",
                            "border": "1px solid #444",
                        },
                        style_data_conditional=[
                            {
                                "if": {"row_index": "odd"},
                                "backgroundColor": "#2a2a2a",
                            },
                            {
                                "if": {"state": "active"},
                                "backgroundColor": "#222",
                                "border": "1px solid #444",
                            },
                        ],
                        css=[{"selector": ".dash-cell.focused",
                              "rule": "background-color: #222 !important;"}],
                        sort_action="native",
                        filter_action="native",
                        page_action="none",
                    )
                ], className="p-2"),
            ], color="dark", outline=True)
        ])
    ], className="mb-3"),

    # ── Raw Messages ──
    dbc.Row([
        dbc.Col([
            dbc.Card([
                dbc.CardHeader("Dernières Trames CAN (brutes)",
                               className="py-1 small fw-bold"),
                dbc.CardBody([
                    html.Pre(id="raw-messages",
                             style={"height": "200px", "overflow": "auto",
                                    "fontSize": "10px", "color": "#0f0",
                                    "backgroundColor": "#111",
                                    "padding": "8px", "margin": "0"}),
                ], className="p-1"),
            ], color="dark", outline=True)
        ])
    ], className="mb-3"),

    # ── Auto-refresh ──
    dcc.Interval(id="interval", interval=UPDATE_INTERVAL_MS, n_intervals=0),

    # ── Hidden stores ──
    dcc.Store(id="recording-state", data=False),
    dcc.Store(id="replay-action-result", data=""),

], fluid=True, className="bg-dark text-light",
   style={"minHeight": "100vh"})


# =============================================================================
# CALLBACKS
# =============================================================================

@app.callback(
    [Output("status-badges", "children"),
     Output("stats-bar", "children"),
     Output("signal-table", "data"),
     Output("signal-count", "children"),
     Output("raw-messages", "children"),
     Output("record-status", "children"),
     # Connection panel outputs
     Output("conn-indicator", "style"),
     Output("conn-state-text", "children"),
     Output("conn-state-text", "style"),
     Output("conn-wifi-ssid", "children"),
     Output("conn-wifi-rssi", "children"),
     Output("conn-wifi-rssi", "style"),
     Output("conn-ping", "children"),
     Output("conn-ping", "style"),
     Output("conn-rate", "children"),
     Output("conn-last-pkt", "children"),
     Output("conn-reconnects", "children"),
     # Replay outputs
     Output("replay-status", "children"),
     Output("replay-progress", "value"),
     Output("replay-progress", "color"),
     Output("btn-replay-play", "disabled"),
     Output("btn-replay-pause", "disabled"),
     Output("btn-replay-stop", "disabled"),
     Output("btn-replay-pause", "children"),
     # Cockpit outputs
     Output("ck-speed", "children"),
     Output("ck-speed", "style"),
     Output("ck-speed-unit", "children"),
     Output("ck-speed-limit", "children"),
     Output("ck-gear", "children"),
     Output("ck-gear", "style"),
     Output("ck-turn-left", "style"),
     Output("ck-turn-right", "style"),
     Output("ck-blind-left", "children"),
     Output("ck-blind-left", "style"),
     Output("ck-blind-right", "children"),
     Output("ck-blind-right", "style"),
     Output("ck-soc", "children"),
     Output("ck-soc", "style"),
     Output("ck-soc-bar", "value"),
     Output("ck-soc-bar", "color"),
     Output("ck-range", "children"),
     Output("ck-power", "children"),
     Output("ck-power", "style"),
     Output("ck-torque", "children"),
     Output("ck-autopilot", "children"),
     Output("ck-autopilot", "style"),
     Output("ck-cruise-speed", "children"),
     Output("ck-lane-depart", "style"),
     Output("ck-fcw", "style"),
     Output("ck-aeb", "style"),
     Output("ck-headlights", "style"),
     Output("ck-highbeam", "style"),
     Output("ck-fog", "style"),
     Output("ck-hazard", "style"),
     Output("ck-brake-light", "style"),
     Output("ck-regen-light", "style"),
     Output("ck-door-fl", "style"),
     Output("ck-door-fr", "style"),
     Output("ck-door-rl", "style"),
     Output("ck-door-rr", "style"),
     Output("ck-trunk", "style"),
     Output("ck-frunk", "style"),
     Output("ck-charge-port", "children"),
     Output("ck-pack-v", "children"),
     Output("ck-pack-a", "children"),
     Output("ck-charge-status", "children"),
     Output("ck-12v", "children"),
     Output("ck-accel-pedal", "children"),
     Output("ck-accel-pedal", "style"),
     Output("ck-brake-pedal", "children"),
     Output("ck-steer-angle", "children"),
     Output("ck-whl-fl", "children"),
     Output("ck-whl-fr", "children"),
     Output("ck-whl-rl", "children"),
     Output("ck-whl-rr", "children"),
     Output("ck-tpms-fl", "children"),
     Output("ck-tpms-fr", "children"),
     Output("ck-tpms-rl", "children"),
     Output("ck-tpms-rr", "children"),
     Output("ck-temp-battery", "children"),
     Output("ck-temp-cabin", "children"),
     Output("ck-temp-exterior", "children"),
     Output("ck-odometer", "children"),
     Output("ck-gps-speed", "children"),
     ] +
    [Output(f"graph-{i}", "figure") for i in range(len(GRAPH_SIGNALS))],
    Input("interval", "n_intervals"),
)
def update_dashboard(_n):
    """Main dashboard update callback."""
    now = time.time()

    with state.lock:
        # Refresh connection state each tick
        state.update_connection_state()

        rx = state.rx_count
        errs = state.decode_errors
        seq_errs = state.sequence_errors
        wifi = state.wifi_connected
        wifi_reconnecting = state.wifi_reconnecting
        udp = state.udp_active
        recording = state.recording
        rec_count = state.record_count
        rec_path = state.blf_path
        uptime = now - state.start_time
        conn_state = state.connection_state
        wifi_rssi = state.wifi_rssi
        wifi_ssid = state.wifi_ssid_current
        ping_ms = state.ping_latency_ms
        ping_ok = state.ping_reachable
        pkt_rate = state.packet_rate
        last_pkt = state.last_packet_time
        reconnects = state.wifi_reconnect_count

        # Signal table data — show ALL signals ever seen (no age filter)
        table_data = []
        for sig_name, (ts, val, unit, msg_name) in state.last_values.items():
            if isinstance(val, float):
                val_str = f"{val:.2f}"
            else:
                val_str = str(val)
            can_id_val = None
            # Find CAN ID from msg name
            for mid, mdef in msg_by_id.items():
                if mdef.name == msg_name:
                    can_id_val = f"0x{mid:03X}"
                    break
            table_data.append({
                "signal": sig_name,
                "value": val_str,
                "unit": unit,
                "msg_name": msg_name,
                "can_id": can_id_val or "",
            })

        # Raw messages (last 30) — with DBC message name + decoded signals
        raw_lines = []
        for msg in list(state.messages)[-30:]:
            data_hex = ' '.join(f'{b:02X}' for b in msg['data_full'][:msg['dlc']])
            # Lookup DBC message name
            dbc_msg = msg_by_id.get(msg['can_id'])
            if dbc_msg:
                msg_label = dbc_msg.name
                # Decode signals for this frame
                try:
                    decoded = dbc_msg.decode(
                        bytes(msg['data_full'][:msg['dlc']]),
                        decode_choices=False
                    )
                    sig_parts = []
                    for sname, sval in decoded.items():
                        if isinstance(sval, float):
                            sig_parts.append(f"{sname}={sval:.2f}")
                        else:
                            sig_parts.append(f"{sname}={sval}")
                    sig_str = " | ".join(sig_parts)
                except Exception:
                    sig_str = ""
                raw_lines.append(
                    f"[{msg['sequence']:05d}] 0x{msg['can_id']:03X} "
                    f"{msg_label:30s} [{msg['dlc']}] {data_hex}"
                    + (f"  → {sig_str}" if sig_str else "")
                )
            else:
                raw_lines.append(
                    f"[{msg['sequence']:05d}] 0x{msg['can_id']:03X} "
                    f"{'(inconnu)':30s} [{msg['dlc']}] {data_hex}"
                )

        # Graphs
        graph_figures = []
        for cat_name, cat_info in GRAPH_SIGNALS.items():
            fig = go.Figure()
            fig.update_layout(
                template="plotly_dark",
                margin=dict(l=40, r=10, t=5, b=25),
                height=195,
                showlegend=True,
                legend=dict(
                    orientation="h", yanchor="top", y=1.12,
                    xanchor="left", x=0, font=dict(size=9)
                ),
                xaxis=dict(
                    range=[now - WINDOW_SECONDS, now],
                    showticklabels=False
                ),
                yaxis=dict(
                    title=dict(text=cat_info["y_label"], font=dict(size=10)),
                    tickfont=dict(size=9)
                ),
                paper_bgcolor="rgba(0,0,0,0)",
                plot_bgcolor="rgba(0,0,0,0.2)",
            )
            for sig_name in cat_info["signals"]:
                if sig_name in state.signals and len(state.signals[sig_name]) > 0:
                    pts = list(state.signals[sig_name])
                    # Filter to window
                    cutoff = now - WINDOW_SECONDS
                    pts = [(t, v) for t, v in pts if t >= cutoff]
                    if pts:
                        times, vals = zip(*pts)
                        fig.add_trace(go.Scattergl(
                            x=list(times), y=list(vals),
                            mode="lines",
                            name=sig_name,
                            line=dict(width=1.5),
                        ))
            graph_figures.append(fig)

    # Status badges
    badges = []
    if conn_state == 'receiving':
        badges.append(make_status_badge("Connecté", "success"))
    elif conn_state == 'stale':
        badges.append(make_status_badge("Signal perdu", "warning"))
    elif conn_state == 'wifi_only':
        badges.append(make_status_badge("WiFi OK / Pas de données", "info"))
    else:
        if wifi_reconnecting:
            badges.append(make_status_badge("Reconnexion...", "warning"))
        else:
            badges.append(make_status_badge("Déconnecté", "danger"))
    if udp and rx > 0:
        badges.append(make_status_badge(f"UDP: {rx}", "success"))
    elif udp:
        badges.append(make_status_badge("UDP: attente", "warning"))
    if recording:
        badges.append(make_status_badge(f"REC: {rec_count}", "danger"))

    # ── Connection panel data ──
    # Indicator color + animation
    conn_colors = {
        'receiving':    '#00ff88',
        'wifi_only':    '#17a2b8',
        'stale':        '#ffc107',
        'disconnected': '#dc3545',
    }
    indicator_color = conn_colors.get(conn_state, '#6c757d')
    # Pulsing animation for stale/reconnecting
    indicator_style = {
        "width": "14px", "height": "14px",
        "borderRadius": "50%",
        "display": "inline-block",
        "verticalAlign": "middle",
        "marginRight": "8px",
        "backgroundColor": indicator_color,
        "boxShadow": f"0 0 8px {indicator_color}",
    }
    if conn_state in ('stale', 'disconnected') or wifi_reconnecting:
        indicator_style["animation"] = "pulse 1.2s ease-in-out infinite"

    conn_labels = {
        'receiving':    'Connecté',
        'wifi_only':    'WiFi seul',
        'stale':        'Signal perdu',
        'disconnected': 'Déconnecté',
    }
    conn_text = conn_labels.get(conn_state, 'Inconnu')
    if wifi_reconnecting:
        conn_text = 'Reconnexion...'
    conn_text_style = {"verticalAlign": "middle", "color": indicator_color}

    # WiFi SSID
    wifi_ssid_text = wifi_ssid or "--"

    # RSSI with color coding
    if wifi_rssi is not None:
        rssi_text = f"{wifi_rssi} dBm"
        if wifi_rssi >= -50:
            rssi_color = {"color": "#00ff88"}
        elif wifi_rssi >= -65:
            rssi_color = {"color": "#ffc107"}
        else:
            rssi_color = {"color": "#dc3545"}
    else:
        rssi_text = "--"
        rssi_color = {"color": "#6c757d"}

    # Ping
    if ping_ms is not None:
        ping_text = f"{ping_ms:.1f} ms"
        ping_color = {"color": "#00ff88"} if ping_ms < 50 else {"color": "#ffc107"}
    elif ping_ok is False and wifi:
        ping_text = "timeout"
        ping_color = {"color": "#dc3545"}
    else:
        ping_text = "--"
        ping_color = {"color": "#6c757d"}

    # Packet rate
    rate_text = f"{pkt_rate:.0f} msg/s" if pkt_rate > 0 else "0 msg/s"

    # Last packet
    if last_pkt > 0:
        age = now - last_pkt
        if age < 1:
            last_pkt_text = "Dernier: <1s"
        elif age < 60:
            last_pkt_text = f"Dernier: {age:.0f}s"
        else:
            last_pkt_text = f"Dernier: {age / 60:.1f}min"
    else:
        last_pkt_text = "Aucun paquet reçu"

    # Reconnect count
    if reconnects > 0:
        reconn_text = f"Reconnexions: {reconnects}"
    else:
        reconn_text = ""

    # Stats bar
    stats_text = (
        f"RX: {rx} | Erreurs: {errs} | "
        f"Pertes seq: {seq_errs} | "
        f"Signaux: {len(table_data)} | "
        f"Uptime: {int(uptime)}s"
    )

    # Signal count
    sig_count_text = f"{len(table_data)} actifs"

    # Raw messages
    raw_text = '\n'.join(reversed(raw_lines)) if raw_lines else "En attente de trames CAN..."

    # Record status
    if recording:
        rec_status = f"Enregistrement en cours... ({rec_count} trames)"
    elif rec_path:
        rec_status = f"Dernier fichier: {os.path.basename(rec_path)}"
    else:
        rec_status = ""

    # ── Replay status ──
    with replay_state.lock:
        rp_active = replay_state.active
        rp_paused = replay_state.paused
        rp_progress = replay_state.progress
        rp_played = replay_state.played_frames
        rp_total = replay_state.total_frames
        rp_elapsed = replay_state.elapsed_s
        rp_duration = replay_state.duration_s
        rp_speed = replay_state.speed
        rp_file = replay_state.file_name

    if rp_active:
        if rp_paused:
            rp_status_text = (f"Pause — {rp_played}/{rp_total} "
                              f"({rp_elapsed:.0f}s/{rp_duration:.0f}s)")
        else:
            rp_status_text = (f"x{rp_speed:g} — {rp_played}/{rp_total} "
                              f"({rp_elapsed:.0f}s/{rp_duration:.0f}s)")
        rp_bar_val = rp_progress * 100
        rp_bar_color = "info" if rp_paused else "success"
        rp_play_disabled = True
        rp_pause_disabled = False
        rp_stop_disabled = False
        rp_pause_label = "Reprendre" if rp_paused else "Pause"
    elif rp_progress >= 1.0 and rp_total > 0:
        rp_status_text = f"Terminé — {rp_total} trames ({rp_duration:.0f}s)"
        rp_bar_val = 100
        rp_bar_color = "warning"
        rp_play_disabled = False
        rp_pause_disabled = True
        rp_stop_disabled = True
        rp_pause_label = "Pause"
    else:
        rp_status_text = ""
        rp_bar_val = 0
        rp_bar_color = "success"
        rp_play_disabled = False
        rp_pause_disabled = True
        rp_stop_disabled = True
        rp_pause_label = "Pause"

    # ══════════════════════════════════════════════════════════════════════════
    # Cockpit data — read last_values for cockpit signals
    # ══════════════════════════════════════════════════════════════════════════
    def ck_val(sig_name, default=None):
        """Get last value of a signal, or default."""
        entry = state.last_values.get(sig_name)
        if entry is not None:
            return entry[1]  # (ts, val, unit, msg_name)
        return default

    def ck_fmt(val, fmt=".1f", suffix=""):
        if val is None:
            return f"--{suffix}"
        try:
            return f"{val:{fmt}}{suffix}"
        except (ValueError, TypeError):
            return f"{val}{suffix}"

    # ── Speed ──
    raw_speed = ck_val("DI_vehicleSpeed") or ck_val("DI_uiSpeed")
    speed_units = ck_val("DI_uiSpeedUnits")
    if raw_speed is not None:
        ck_speed_text = f"{abs(raw_speed):.0f}"
        speed_color = "#fff"
    else:
        ck_speed_text = "---"
        speed_color = "#555"
    ck_speed_style = {"fontSize": "64px", "fontWeight": "bold",
                      "textAlign": "center", "color": speed_color,
                      "lineHeight": "1", "fontFamily": "monospace"}
    ck_speed_unit_text = "mph" if speed_units and speed_units == 1 else "km/h"

    # Speed limit
    sl = ck_val("DAS_fusedSpeedLimit") or ck_val("UI_mapSpeedLimit")
    ck_speed_limit_text = f"{sl:.0f}" if sl is not None else "--"

    # ── Gear ──
    gear_val = ck_val("DI_gear")
    gear_map = {1: "P", 2: "R", 3: "N", 4: "D",
                "SNA": "P", 0: "?"}
    gear_text = gear_map.get(gear_val, str(gear_val) if gear_val is not None
                             else "P")
    gear_colors = {"P": "#888", "R": "#ff4444", "N": "#ffaa00",
                   "D": "#00cc66", "?": "#555"}
    ck_gear_style = {"fontSize": "20px", "fontWeight": "bold",
                     "textAlign": "center",
                     "color": gear_colors.get(gear_text, "#888"),
                     "marginBottom": "2px"}

    # ── Turn signals ──
    turn_stalk = ck_val("SCCM_turnIndicatorStalkStatus")
    # 1=left, 2=right, 3=hazard typically
    turn_left_active = turn_stalk in (1, 3)
    turn_right_active = turn_stalk in (2, 3)
    left_turn_style = {"fontSize": "36px", "textAlign": "center",
                       "color": "#ff8800" if turn_left_active else "#333"}
    if turn_left_active:
        left_turn_style["animation"] = "pulse 0.6s ease-in-out infinite"
    right_turn_style = {"fontSize": "36px", "textAlign": "center",
                        "color": "#ff8800" if turn_right_active else "#333"}
    if turn_right_active:
        right_turn_style["animation"] = "pulse 0.6s ease-in-out infinite"

    # ── Blind spots ──
    bsl = ck_val("DAS_blindSpotRearLeft")
    bsr = ck_val("DAS_blindSpotRearRight")
    bsl_active = bsl is not None and bsl not in (0, False, "SNA")
    bsr_active = bsr is not None and bsr not in (0, False, "SNA")
    ck_blind_l_text = "⚠ DÉTECTÉ" if bsl_active else "--"
    ck_blind_l_style = {"fontSize": "14px", "textAlign": "center",
                        "color": "#ff4444" if bsl_active else "#555",
                        "fontWeight": "bold" if bsl_active else "normal"}
    ck_blind_r_text = "⚠ DÉTECTÉ" if bsr_active else "--"
    ck_blind_r_style = {"fontSize": "14px", "textAlign": "center",
                        "color": "#ff4444" if bsr_active else "#555",
                        "fontWeight": "bold" if bsr_active else "normal"}

    # ── SOC / Range ──
    soc = ck_val("SOCUI292") or ck_val("SOCave292") or ck_val("UI_SOC")
    if soc is not None:
        ck_soc_text = f"{soc:.0f}%"
        if soc > 50:
            soc_color = "#00cc66"
            soc_bar_color = "success"
        elif soc > 20:
            soc_color = "#ffaa00"
            soc_bar_color = "warning"
        else:
            soc_color = "#ff4444"
            soc_bar_color = "danger"
        soc_bar_val = min(100, max(0, soc))
    else:
        ck_soc_text = "--%"
        soc_color = "#555"
        soc_bar_color = "secondary"
        soc_bar_val = 0
    ck_soc_style = {"fontSize": "28px", "fontWeight": "bold",
                    "textAlign": "center", "color": soc_color}

    ui_range_mi = ck_val("UI_Range") or ck_val("UI_idealRange")
    if ui_range_mi is not None:
        ui_range_km = ui_range_mi * 1.60934  # miles → km
        ck_range_text = f"{ui_range_km:.0f} km"
    else:
        ck_range_text = "-- km"

    # ── Power / Torque ──
    rear_torque = ck_val("RearTorque1D8")
    front_torque = ck_val("FrontTorque1D5")
    total_torque = None
    if rear_torque is not None or front_torque is not None:
        total_torque = (rear_torque or 0) + (front_torque or 0)

    elec_power = ck_val("BMS_hvacPowerRequest")
    pack_v = ck_val("BattVoltage132")
    pack_a = ck_val("SmoothBattCurrent132")
    rear_power = ck_val("RearPower266")
    front_power = ck_val("FrontPower2E5")
    if rear_power is not None or front_power is not None:
        power_kw = (rear_power or 0) + (front_power or 0)
    elif pack_v is not None and pack_a is not None:
        power_kw = (pack_v * pack_a) / 1000.0
    elif elec_power is not None:
        power_kw = elec_power
    else:
        power_kw = None

    if power_kw is not None:
        pw_sign = "+" if power_kw > 0 else ""
        ck_power_text = f"{pw_sign}{power_kw:.1f} kW"
        pw_color = "#00cc66" if power_kw <= 0 else "#ff6666"
    else:
        ck_power_text = "-- kW"
        pw_color = "#555"
    ck_power_style = {"fontSize": "22px", "fontWeight": "bold",
                      "textAlign": "center", "color": pw_color}

    ck_torque_text = (f"{total_torque:.0f} Nm"
                      if total_torque is not None else "-- Nm")

    # ── Autopilot ──
    ap_state = ck_val("DAS_autopilotState")
    ap_hands = ck_val("DAS_autopilotHandsOnState")
    cruise_set = ck_val("DAS_setSpeed")

    ap_labels = {0: "OFF", 1: "STANDBY", 2: "ACTIF", 3: "ACTIF+",
                 4: "OVERRIDE", 5: "ERREUR", 14: "DÉSACTIVÉ", 15: "ERREUR"}
    ap_text = ap_labels.get(ap_state, str(ap_state) if ap_state is not None
                            else "OFF")
    ap_active = ap_state in (2, 3)
    ap_color = "#3399ff" if ap_active else (
        "#ffaa00" if ap_state == 1 else "#555")
    ck_ap_style = {"fontSize": "16px", "fontWeight": "bold",
                   "textAlign": "center", "color": ap_color}
    if ap_active and ap_hands and ap_hands > 1:
        ap_text += " ⚠ MAINS!"
        ck_ap_style["color"] = "#ff4444"

    ck_cruise_text = (f"Consigne: {cruise_set:.0f} km/h"
                      if cruise_set is not None else "")

    # LDW / FCW / AEB indicators
    ldw_val = ck_val("DAS_laneDepartureWarning")
    fcw_val = ck_val("DAS_forwardCollisionWarning")
    aeb_val = ck_val("DAS_aebEvent")

    def _adas_style(val, base_color="#00cc66"):
        active = val is not None and val not in (0, False, "SNA")
        return {"fontSize": "10px",
                "color": "#ff4444" if active else (
                    base_color if val is not None else "#444"),
                "fontWeight": "bold" if active else "normal",
                "marginRight": "6px"}

    ck_ldw_style = _adas_style(ldw_val)
    ck_fcw_style = _adas_style(fcw_val)
    ck_aeb_style = _adas_style(aeb_val)
    ck_aeb_style.pop("marginRight", None)

    # ── Lights ──
    light_sw = ck_val("UI_lightSwitch")
    high_beam = ck_val("SCCM_highBeamStalkStatus")
    front_fog = ck_val("UI_frontFogSwitch")
    hazard = ck_val("VCLEFT_hazardButtonPressed")
    brake_lamp = ck_val("ESP_brakeLamp") or ck_val("VCLEFT_brakeLightStatus")
    regen_light = ck_val("DI_regenLight")

    def _light_icon_style(base_style, active):
        s = dict(base_style)
        s["opacity"] = "1.0" if active else "0.3"
        return s

    hl_base = {"fontSize": "18px", "marginRight": "8px"}
    ck_headlights_style = _light_icon_style(
        hl_base, light_sw is not None and light_sw not in (0, False, "OFF"))
    ck_highbeam_style = _light_icon_style(
        hl_base, high_beam is not None and high_beam not in (0, False, "SNA"))
    ck_fog_style = _light_icon_style(
        hl_base, front_fog is not None and front_fog not in (0, False))
    ck_hazard_style = _light_icon_style(
        {"fontSize": "18px"},
        hazard is not None and hazard not in (0, False))
    if hazard and hazard not in (0, False):
        ck_hazard_style["animation"] = "pulse 0.5s ease-in-out infinite"

    brake_active = brake_lamp is not None and brake_lamp not in (0, False,
                                                                  "SNA")
    ck_brake_style = {"fontSize": "10px",
                      "color": "#ff4444" if brake_active else "#444",
                      "fontWeight": "bold" if brake_active else "normal",
                      "marginRight": "8px"}
    regen_active = regen_light is not None and regen_light not in (0, False)
    ck_regen_style = {"fontSize": "10px",
                      "color": "#00cc66" if regen_active else "#444",
                      "fontWeight": "bold" if regen_active else "normal"}

    # ── Doors ──
    def _door_style(sig_name):
        val = ck_val(sig_name)
        is_open = val is not None and val not in (0, False, "SNA",
                                                   "CLOSED", "closed")
        return {"fontSize": "11px",
                "color": "#ff4444" if is_open else "#444",
                "margin": "0 4px", "padding": "2px 4px",
                "borderRadius": "3px",
                "border": "1px solid " + ("#ff4444" if is_open else "#333"),
                "backgroundColor": "#3a1111" if is_open else "transparent",
                "fontWeight": "bold" if is_open else "normal"}

    ck_door_fl_style = _door_style("VCLEFT_frontDoorState")
    ck_door_fr_style = _door_style("VCRIGHT_frontLatchStatus")
    ck_door_rl_style = _door_style("VCLEFT_rearDoorState")
    ck_door_rr_style = _door_style("VCRIGHT_rearLatchStatus")

    trunk_val = ck_val("VCRIGHT_trunkLatchStatus")
    trunk_open = trunk_val is not None and trunk_val not in (0, False, "SNA",
                                                              "CLOSED")
    ck_trunk_style = {"fontSize": "10px",
                      "color": "#ff8800" if trunk_open else "#444",
                      "marginRight": "8px",
                      "fontWeight": "bold" if trunk_open else "normal"}

    frunk_val = ck_val("VCLEFT_liftgateState")
    frunk_open = frunk_val is not None and frunk_val not in (0, False, "SNA",
                                                              "CLOSED")
    ck_frunk_style = {"fontSize": "10px",
                      "color": "#ff8800" if frunk_open else "#444",
                      "fontWeight": "bold" if frunk_open else "normal"}

    # Charge port
    cp_door = ck_val("CP_chargeDoorOpen") or ck_val("CP_chargeDoorOpenUI")
    cp_cable = ck_val("CP_chargeCableState")
    if cp_door and cp_door not in (0, False, "SNA"):
        if cp_cable and cp_cable not in (0, False, "SNA"):
            ck_charge_port_text = "Port: Ouvert + Câble"
        else:
            ck_charge_port_text = "Port: Ouvert"
    else:
        ck_charge_port_text = "Port: Fermé"

    # ── Battery HV ──
    pv = ck_val("BattVoltage132")
    pa = ck_val("SmoothBattCurrent132")
    ck_pack_v_text = f"{pv:.0f} V" if pv is not None else "-- V"
    ck_pack_a_text = f"{pa:.1f} A" if pa is not None else "-- A"

    charge_st = ck_val("BMS_uiChargeStatus")
    charge_map = {0: "Pas en charge", 1: "En charge", 2: "Charge terminée",
                  3: "Erreur charge"}
    ck_charge_st_text = charge_map.get(charge_st,
                                       str(charge_st)
                                       if charge_st is not None else "--")

    v12 = ck_val("v12vBattVoltage261")
    ck_12v_text = f"12V: {v12:.1f}V" if v12 is not None else "12V: --"

    # ── Pedals ──
    accel = ck_val("DI_accelPedalPos")
    ck_accel_text = f"{accel:.0f}%" if accel is not None else "0%"
    accel_pct = accel if accel is not None else 0
    accel_color = ("#00cc66" if accel_pct < 50
                   else ("#ffaa00" if accel_pct < 80 else "#ff4444"))
    ck_accel_style = {"fontSize": "14px", "textAlign": "center",
                      "color": accel_color}

    brake_pedal = ck_val("DI_brakePedalState") or ck_val(
        "VCLEFT_brakePressed")
    brake_applied = (brake_pedal is not None
                     and brake_pedal not in (0, False, "SNA"))
    ck_brake_pedal_text = ("Frein: APPUYÉ" if brake_applied
                           else "Frein: --")

    # ── Steering ──
    steer = ck_val("SteeringAngle129")
    ck_steer_text = f"{steer:.1f}°" if steer is not None else "0.0°"

    # ── Wheel speeds ──
    ck_wfl = ck_fmt(ck_val("WheelSpeedFL175"), ".0f")
    ck_wfr = ck_fmt(ck_val("WheelSpeedFR175"), ".0f")
    ck_wrl = ck_fmt(ck_val("WheelSpeedRL175"), ".0f")
    ck_wrr = ck_fmt(ck_val("WheelSpeedRR175"), ".0f")

    # ── TPMS ──
    def _tpms_fmt(sig):
        v = ck_val(sig)
        if v is not None:
            return f"{v:.1f}"
        return "--"

    ck_tfl = _tpms_fmt("TPMSFLpressure31F")
    ck_tfr = _tpms_fmt("TPMSFRpressure31F")
    ck_trl = _tpms_fmt("TPMSRLpressure31F")
    ck_trr = _tpms_fmt("TPMSRRpressure31F")

    # ── Temperatures ──
    bat_temp = (ck_val("BMSmaxPackTemperature")
                or ck_val("BMSminPackTemperature"))
    cabin_temp = ck_val("VCRIGHT_hvacDuctTargetLeft")
    ext_temp = ck_val("VCFRONT_tempAmbient")

    ck_tbat = (f"Batt: {bat_temp:.1f}°C" if bat_temp is not None
               else "Batt: --°C")
    ck_tcab = (f"Hab: {cabin_temp:.0f}°C" if cabin_temp is not None
               else "Hab: --°C")
    ck_text = (f"Ext: {ext_temp:.1f}°C" if ext_temp is not None
               else "Ext: --°C")

    # ── Odometer / GPS ──
    # UI_odometer (0x3F3, factor 0.1, "km") is the reliable source.
    # Odometer3B6 (0x3B6, factor 0.001) often decodes to absurd values.
    odo = ck_val("UI_odometer") or ck_val("Odometer3B6")
    ck_odo_text = (f"{odo:,.0f} km" if odo is not None else "-- km")
    gps_spd = ck_val("UI_gpsVehicleSpeed")
    ck_gps_text = (f"GPS: {gps_spd:.0f} km/h" if gps_spd is not None
                   else "GPS: -- km/h")

    return [badges, stats_text, table_data, sig_count_text, raw_text,
            rec_status,
            # Connection panel
            indicator_style, conn_text, conn_text_style,
            wifi_ssid_text, rssi_text, rssi_color,
            ping_text, ping_color, rate_text,
            last_pkt_text, reconn_text,
            # Replay
            rp_status_text, rp_bar_val, rp_bar_color,
            rp_play_disabled, rp_pause_disabled, rp_stop_disabled,
            rp_pause_label,
            # Cockpit
            ck_speed_text, ck_speed_style, ck_speed_unit_text,
            ck_speed_limit_text,
            gear_text, ck_gear_style,
            left_turn_style, right_turn_style,
            ck_blind_l_text, ck_blind_l_style,
            ck_blind_r_text, ck_blind_r_style,
            ck_soc_text, ck_soc_style, soc_bar_val, soc_bar_color,
            ck_range_text,
            ck_power_text, ck_power_style, ck_torque_text,
            ap_text, ck_ap_style, ck_cruise_text,
            ck_ldw_style, ck_fcw_style, ck_aeb_style,
            ck_headlights_style, ck_highbeam_style, ck_fog_style,
            ck_hazard_style,
            ck_brake_style, ck_regen_style,
            ck_door_fl_style, ck_door_fr_style,
            ck_door_rl_style, ck_door_rr_style,
            ck_trunk_style, ck_frunk_style,
            ck_charge_port_text,
            ck_pack_v_text, ck_pack_a_text, ck_charge_st_text, ck_12v_text,
            ck_accel_text, ck_accel_style, ck_brake_pedal_text,
            ck_steer_text,
            ck_wfl, ck_wfr, ck_wrl, ck_wrr,
            ck_tfl, ck_tfr, ck_trl, ck_trr,
            ck_tbat, ck_tcab, ck_text,
            ck_odo_text, ck_gps_text,
            ] + graph_figures


@app.callback(
    [Output("btn-record", "disabled"),
     Output("btn-stop", "disabled"),
     Output("recording-state", "data")],
    [Input("btn-record", "n_clicks"),
     Input("btn-stop", "n_clicks")],
    State("recording-state", "data"),
    prevent_initial_call=True,
)
def toggle_recording(rec_clicks, stop_clicks, is_recording):
    """Handle record/stop button clicks."""
    ctx = callback_context
    if not ctx.triggered:
        return False, True, False

    button_id = ctx.triggered[0]["prop_id"].split(".")[0]

    if button_id == "btn-record" and not is_recording:
        if start_recording():
            return True, False, True
    elif button_id == "btn-stop" and is_recording:
        stop_recording()
        return False, True, False

    return is_recording, not is_recording, is_recording


# ─── Replay controls callback ────────────────────────────────────────────────

@app.callback(
    Output("replay-action-result", "data"),
    [Input("btn-replay-play", "n_clicks"),
     Input("btn-replay-pause", "n_clicks"),
     Input("btn-replay-stop", "n_clicks")],
    [State("replay-file-select", "value"),
     State("replay-speed", "value")],
    prevent_initial_call=True,
)
def handle_replay_controls(play_clicks, pause_clicks, stop_clicks,
                           selected_file, speed_str):
    """Handle replay Play/Pause/Stop button clicks."""
    ctx = callback_context
    if not ctx.triggered:
        return ""

    button_id = ctx.triggered[0]["prop_id"].split(".")[0]

    if button_id == "btn-replay-play":
        if not selected_file:
            return "no-file"
        speed = float(speed_str) if speed_str else 1.0
        start_replay(selected_file, speed)
        return "started"

    elif button_id == "btn-replay-pause":
        pause_replay()
        return "paused"

    elif button_id == "btn-replay-stop":
        stop_replay()
        return "stopped"

    return ""


@app.callback(
    Output("replay-file-select", "options"),
    Input("btn-replay-refresh", "n_clicks"),
    prevent_initial_call=True,
)
def refresh_replay_files(_n):
    """Refresh the list of available BLF files."""
    return list_recordings()


# =============================================================================
# MAIN
# =============================================================================

def main():
    print("\n" + "=" * 50)
    print("  Tesla CAN Dashboard v1.0")
    print("  ESP32 WiFi Bridge -> UDP -> Dash Plotly")
    print("=" * 50 + "\n")

    # Auto-connect to WiFi
    print("[1/4] Connexion WiFi...")
    wifi_ok = auto_connect_wifi()
    if not wifi_ok:
        print(f"[WiFi] Impossible de se connecter à '{WIFI_SSID}'")
        print("[WiFi] La reconnexion automatique est active.\n")

    # Start WiFi watchdog thread
    print("[2/4] Démarrage watchdog WiFi (reconnexion auto)...")
    watchdog = threading.Thread(target=wifi_watchdog_thread, daemon=True)
    watchdog.start()

    # Start UDP receiver thread
    print("[3/4] Démarrage récepteur UDP...")
    udp_thread = threading.Thread(target=udp_receiver_thread, daemon=True)
    udp_thread.start()

    # Start dashboard
    print(f"[4/4] Dashboard: http://localhost:{DASH_PORT}\n")
    print("=" * 50)
    print(f"  Ouvrir: http://localhost:{DASH_PORT}")
    print(f"  WiFi: {WIFI_SSID} / {WIFI_PASSWORD}")
    print(f"  UDP: port {UDP_PORT}")
    print(f"  DBC: {DBC_PATH.name} ({len(msg_by_id)} messages)")
    print("=" * 50 + "\n")

    app.run(host=DASH_HOST, port=DASH_PORT, debug=False)


if __name__ == "__main__":
    main()
