# TeslaCam

Caméra de recul sans fil pour Tesla — deux ESP32-S3 communiquant en Wi-Fi direct (SoftAP + UDP).

```
┌─────────────────────┐      Wi-Fi SoftAP       ┌─────────────────────┐
│  ESP32-S3 Camera    │───── UDP :5000 ─────────▶│  ESP32-S3 Écran     │
│  Freenove + OV5640  │   JPEG chunked           │  JC3636W518C        │
│  SoftAP 192.168.4.1 │   ~15 KB/frame           │  ST77916 360×360    │
│  HVGA 480×320       │   12.5 fps               │  Station 192.168.4.2│
└─────────────────────┘                          └─────────────────────┘
```

## Matériel

| Module | Rôle | Processeur | Capteur / Écran | PSRAM |
|--------|------|-----------|-----------------|-------|
| Freenove ESP32-S3 WROOM | Camera | ESP32-S3 | OV5640 | 8 MB OPI |
| JC3636W518C | Écran | ESP32-S3 | ST77916 QSPI 360×360 rond | 8 MB OPI |

## Architecture

- **Camera** : SoftAP Wi-Fi + capture JPEG OV5640 + envoi UDP unicast sur core 1
- **Écran** : Station Wi-Fi + réception UDP bloquante sur core 0 + décodage JPEG + affichage batch sur core 1
- **Protocole** : paquets UDP de 8B header + ≤1400B payload, réassemblés par `frame_id`
- **Triple buffer** côté Écran (réception / prêt / affichage) avec mutex FreeRTOS
- **Screen buffer** PSRAM 360×360 : le callback JPEG écrit en mémoire, puis un seul transfert QSPI par frame

## Performances (v1.1)

| Métrique | Valeur |
|----------|--------|
| FPS Camera | 12.5 |
| FPS Écran | 12.5 |
| Latence | ~80 ms |
| Taille JPEG | ~15 KB |
| Résolution capture | HVGA 480×320 |
| Affichage | 360×360 center-crop |

## Build & Flash

Prérequis : [PlatformIO](https://platformio.org/)

```bash
# Camera
cd Camera && pio run -t upload

# Écran
cd Ecran/firmware && pio run -t upload
```

Les ports USB sont configurés dans les `platformio.ini` respectifs.

## Structure

```
Camera/
  src/main.cpp          # SoftAP + UDP sender + OV5640
  platformio.ini
Ecran/firmware/
  src/main.cpp          # Station + UDP recv task + JPEGDEC + ST77916
  platformio.ini
  partitions.csv
PRD.md                  # Product Requirements Document
```

## Dépendances

**Camera** : `esp_camera` (intégré ESP-IDF), lwip sockets

**Écran** :
- `moononournation/GFX Library for Arduino` — driver ST77916 QSPI
- `bitbank2/JPEGDEC` — décodage JPEG matériel

## Historique

- **v1.0** — Streaming fonctionnel (Camera 12.5 fps, Écran 6.7 fps)
- **v1.1** — Triple buffer + task UDP dédiée + screen buffer batch → Écran 12.5 fps
