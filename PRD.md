# PRD — TeslaCam : Streaming Photo Camera → Écran

## 1. Résumé du projet

Système embarqué de caméra de recul pour Tesla composé de deux ESP32-S3 communiquant en Wi-Fi. L'ESP32-S3 **Camera** capture une photo JPEG 360×360 via le capteur OV5640, puis l'envoie à l'ESP32-S3 **Écran** qui l'affiche sur un écran rond ST77916 (360×360 pixels).

---

## 2. Matériel

| Composant | Module | Caractéristiques clés |
|---|---|---|
| **Camera** | Freenove ESP32-S3 WROOM | ESP32-S3, OV5640, 8 MB PSRAM OPI, Wi-Fi 802.11n |
| **Écran** | JC3636W518C | ESP32-S3, ST77916 QSPI, 360×360 rond, 16 MB Flash, 8 MB PSRAM OPI |

---

## 3. Architecture

```
┌───────────────────────┐         Wi-Fi (SoftAP)        ┌───────────────────────┐
│   ESP32-S3 Camera     │ ────── UDP port 5000 ──────── │   ESP32-S3 Écran      │
│                       │                                │                       │
│  OV5640 (JPEG 360x360)│  Paquets UDP chunked          │  ST77916 360x360 rond │
│  Mode : SoftAP        │  Header 8B + payload ≤1452B   │  Mode : Station       │
│  IP : 192.168.4.1     │                                │  IP : 192.168.4.2     │
│                       │                                │  LVGL + JPEGDEC       │
└───────────────────────┘                                └───────────────────────┘
```

### Choix de communication : SoftAP + UDP raw

Basé sur l'analyse du Guide.txt, le protocole **SoftAP + UDP raw** est retenu car :

- **Débit** : ~20 Mbps effectifs en UDP sur Wi-Fi 802.11n (vs ~1 Mbps ESP-NOW)
- **Latence** : < 80 ms (vs 200–400 ms en HTTP/MJPEG)
- **Autonomie** : pas de routeur Wi-Fi externe nécessaire
- **Résolution cible** : 360×360 JPEG (~15–20 KB/frame) → largement dans la bande passante

---

## 4. Exigences fonctionnelles

### 4.1 ESP32-S3 Camera (émetteur)

| ID | Exigence | Détail |
|---|---|---|
| CAM-01 | Capturer des photos JPEG 360×360 | Utiliser la compression JPEG native de l'OV5640. Framesize personnalisé ou crop depuis HVGA. |
| CAM-02 | Créer un Access Point Wi-Fi | SSID : `TeslaCam`, pas de mot de passe (réseau local fermé). IP : `192.168.4.1`. |
| CAM-03 | Envoyer les frames via UDP chunked | Découper chaque JPEG en chunks ≤ 1452 bytes avec header 8 bytes. Port destination : `5000`. |
| CAM-04 | Cadence cible | ≥ 15 fps en continu. |
| CAM-05 | Miroir horizontal + flip vertical | Image retournée pour montage caméra de recul (configurable). |
| CAM-06 | Task FreeRTOS dédiée | Le streaming tourne sur un core dédié (core 1) pour ne pas bloquer le système. |

### 4.2 ESP32-S3 Écran (récepteur)

| ID | Exigence | Détail |
|---|---|---|
| ECR-01 | Se connecter au SoftAP Camera | Station Wi-Fi connectée à `TeslaCam`, IP statique `192.168.4.2`. |
| ECR-02 | Recevoir et réassembler les frames UDP | Écouter sur le port `5000`. Réassembler les chunks par `frame_id`. Drop des frames incomplètes ou en retard. |
| ECR-03 | Décoder le JPEG | Utiliser la librairie JPEGDEC pour décoder le JPEG en RGB565. |
| ECR-04 | Afficher sur l'écran ST77916 | Rendu via Arduino_GFX `draw16bitRGBBitmap()`. Pas d'LVGL nécessaire pour l'affichage photo (rendu direct). |
| ECR-05 | Timeout / écran noir | Si aucune frame complète reçue pendant > 500 ms, afficher un écran noir (ne jamais rester sur une image figée). |
| ECR-06 | Double buffering frames | Deux buffers JPEG en PSRAM : un en réception, un en décodage, pour éviter les artefacts. |

---

## 5. Protocole UDP — Structure des paquets

```
┌─────────────────────────────────────────┐
│  HEADER  (8 bytes)                      │
│  frame_id     : uint16_t  (2B)          │
│  chunk_id     : uint16_t  (2B)          │
│  total_chunks : uint16_t  (2B)          │
│  chunk_size   : uint16_t  (2B)          │
├─────────────────────────────────────────┤
│  PAYLOAD (≤ 1452 bytes)                 │
│  données JPEG brutes                    │
└─────────────────────────────────────────┘
```

- **MTU Wi-Fi** : 1460 bytes utiles en UDP → 8 header + 1452 payload
- **Frame JPEG 360×360** estimée : ~12–20 KB → ~9–14 chunks par frame

---

## 6. Configuration OV5640

```cpp
sensor_t *s = esp_camera_sensor_get();
s->set_framesize(s, FRAMESIZE_HVGA);   // 480×320, puis crop/resize à 360×360
s->set_quality(s, 10);                  // Qualité JPEG (bon compromis taille/qualité)
s->set_pixformat(s, PIXFORMAT_JPEG);
s->set_vflip(s, 1);                    // Flip vertical (caméra de recul)
s->set_hmirror(s, 1);                  // Miroir horizontal
```

> **Note** : L'OV5640 ne supporte pas nativement le 360×360. Stratégie : capturer en HVGA (480×320) ou VGA (640×480) en JPEG, puis laisser le récepteur décoder et afficher centré/croppé sur le 360×360.

---

## 7. Performances attendues

| Métrique | Valeur cible |
|---|---|
| Résolution capture | HVGA 480×320 (crop à 360×360 côté écran) |
| Taille JPEG moyenne | ~15–20 KB |
| FPS | ≥ 15 fps |
| Latence bout-en-bout | < 100 ms |
| Portée Wi-Fi | ~10 m (intérieur véhicule, largement suffisant) |
| Consommation Camera | ~350 mA @ 3.3V |

---

## 8. Stack technique

### Camera (`Camera/`)

| Composant | Choix |
|---|---|
| Framework | Arduino (ESP-IDF sous-jacent) |
| Platform | espressif32 (PlatformIO) |
| Wi-Fi | Mode SoftAP natif ESP-IDF |
| Transport | AsyncUDP (ou `lwip_sendto` raw) |
| Capture | `esp_camera` API |

### Écran (`Ecran/firmware/`)

| Composant | Choix |
|---|---|
| Framework | Arduino (pioarduino ESP32 core) |
| Platform | espressif32 via pioarduino |
| Display driver | Arduino_GFX (ST77916 QSPI) |
| Décodage JPEG | JPEGDEC (bitbank2) |
| Wi-Fi | Mode Station |
| Transport | AsyncUDP (écoute port 5000) |

### Dépendances à ajouter

**Camera** `platformio.ini` :
```ini
lib_deps =
    ESP Async UDP
```

**Écran** `platformio.ini` :
```ini
lib_deps =
    moononournation/GFX Library for Arduino@^1.5.0
    bitbank2/JPEGDEC@^1.4.2
    ESP Async UDP
```

> LVGL n'est plus nécessaire pour la phase d'affichage photo (rendu direct GFX). Il pourra être réintégré plus tard pour une UI complète.

---

## 9. Plan d'implémentation

### Phase 1 — Communication de base
- [ ] **Camera** : Remplacer le mode Station + HTTP par un mode SoftAP + UDP sender
- [ ] **Écran** : Remplacer la démo arc gauge par un récepteur UDP + affichage JPEG
- [ ] Valider la réception d'une frame JPEG complète (log Serial)

### Phase 2 — Affichage fonctionnel
- [ ] **Écran** : Décoder les JPEG reçus via JPEGDEC et afficher via `draw16bitRGBBitmap()`
- [ ] **Écran** : Implémenter le crop/centrage de l'image HVGA vers 360×360
- [ ] **Écran** : Implémenter le timeout (écran noir après 500 ms sans frame)

### Phase 3 — Optimisation
- [ ] Mesurer et optimiser le FPS et la latence
- [ ] Ajuster la qualité JPEG et la résolution pour le meilleur compromis

---

## 10. Risques et mitigations

| Risque | Impact | Mitigation |
|---|---|---|
| Perte de paquets UDP | Artefacts visuels, frames incomplètes | Drop des frames incomplètes, toujours afficher la dernière frame valide |
| OV5640 ne fait pas 360×360 natif | Image non carrée | Capturer en HVGA/VGA puis crop côté récepteur |
| Interférences Wi-Fi dans le véhicule | Baisses de FPS | Canal Wi-Fi fixe (1 ou 11), puissance TX maximale |
| Dépassement mémoire PSRAM | Crash | Limiter à 2 buffers, surveiller `heap_caps_get_free_size()` |

---

## 11. Critères de validation

- [ ] L'image de la caméra s'affiche sur l'écran rond en < 100 ms de latence
- [ ] Le FPS mesuré est ≥ 15 fps en conditions normales
- [ ] L'écran passe au noir si la caméra est déconnectée (timeout 500 ms)
- [ ] Le système fonctionne sans routeur Wi-Fi externe (SoftAP autonome)
- [ ] Pas de fuite mémoire après 1h de fonctionnement continu
