# COVERAGE-AI — setup guide

**Flow:** ESP32-S3 bot (RSSI of the AP, UDP) + overhead cameras (ArUco → homography → x,y) → `server.py` pairs each RSSI sample with the bot position → fits the AP location (path-loss) + Gaussian Process on the residuals → heatmap, uncertainty, weak zones, repeater suggestion → browser dashboard.

## 1. Try it with no hardware
```
pip install -r requirements.txt
python server.py --sim
```
Open http://localhost:8000. The simulator drives a fake bot with a hidden AP at (1.0, 2.2) and a "wall" at x > 2.6. After ~20 s you should see the heatmap, the AP star near the true spot, and a repeater suggestion.

## 2. Wiring (L298N + ESP32-S3)
| L298N | ESP32-S3 |
|---|---|
| ENA | GPIO4 (remove the ENA jumper) |
| IN1 / IN2 | GPIO5 / GPIO6 |
| ENB | GPIO7 (remove the ENB jumper) |
| IN3 / IN4 | GPIO15 / GPIO16 |
| GND | GND (common ground is essential) |

Motor battery (7–12 V) → L298N 12V. Power the ESP32-S3 from a USB power bank, not from the L298N 5V pin. If a wheel spins the wrong way, swap that motor's two wires.

## 3. Flash the firmware
1. Arduino IDE → Boards Manager → install **esp32 by Espressif, version 3.x**.
2. Board: **ESP32S3 Dev Module**. Open `firmware/coverage_esp32s3/coverage_esp32s3.ino`.
3. Edit `SSID`, `PASS` (the AP you want to map) and `PC` (laptop IP; find it with `ipconfig` / `ip a`). Laptop must be on the same network.
4. Upload. Serial Monitor (115200) should print the IP, channel and BSSID.

Note: the bot measures the AP it is *connected to*. If you have a mesh or several APs with one SSID, the bot may roam: set `TARGET_BSSID` in `server.py` to keep only one.

## 4. Cameras and ArUco
1. Print ArUco markers from dictionary **4x4_50**: static ids 0–3 (or more) and bot id 10.
2. Fix the static markers flat on the floor at known positions. Measure each marker's **center** in metres from your chosen origin (x right, y down as the camera sees it).
3. Put marker 10 flat on top of the bot, parallel to the floor.
4. Edit `server.py` config: `W, H`, `CAMS` (RTSP URLs), `STATIC`, `BOT_ID`.
5. Each camera needs **at least 4 static markers in view** for its homography. With two cameras covering halves, use 6–8 static markers and list them all in `STATIC` (shared markers in the overlap give a consistent frame).
6. Reduce camera latency: lower the stream's resolution/bitrate, use the sub-stream, keep the bot slow (~0.1–0.2 m/s).

## 5. Run a real survey
1. Power the bot; confirm the ESP IP appears in the dashboard.
2. `python server.py`, open http://localhost:8000.
3. Check the bot position updates (red dot). If it says "not seen", the camera doesn't see ≥4 static markers or the bot marker.
4. Drive with W/A/S/D (hold keys). Cover the area in lawnmower passes, slowly. The model retrains every ~25 new samples.
5. Watch the **Uncertainty** view and drive into high-σ areas.
6. Read results: white dotted line = weak-zone boundary (slider), star = estimated AP, green diamond = suggested repeater.
7. **Export CSV** gives `time,x,y,rssi,bssid,channel`. Raw log is also in `log.csv`.

## 6. Tuning and troubleshooting
- Noisy map: drive slower, or raise `SEND_MS` averaging (the server bins samples per 10 cm cell).
- Motors weak/stalling: PWM below ~100 usually doesn't move the bot; raise the speeds in `dashboard.html`.
- No samples: check the firewall allows UDP 5005 inbound; check `PC` IP.
- Dashboard blank offline: Plotly loads from a CDN; download `plotly-2.35.2.min.js` and point the script tag at it.
- AP estimate off: it assumes one dominant AP and simple path loss; walls shift it. Trust the GP map more than the star.
- Repeater suggestion assumes a repeater with the same transmit power as the AP and ignores walls.

## 7. Not included yet
Autonomous waypoint/frontier driving, camera video in the dashboard, spatial cross-validation scores, multi-AP mapping.
