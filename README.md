# Sanjeevani Rover — AI Search-and-Rescue Recon Rover[cite: 9, 16]

**SEVA First Innovation Challenge 2026** | **Indorama Public School**[cite: 16]  
**Team:** Divyanshu, Saloni, Anika, and Ashutosh[cite: 16]  
**Readiness Level:** TRL 5 (Prototype tested in an environment close to real use)[cite: 20]

When an earthquake, flood, or building collapse traps people, the first hours decide who lives, and human rescuers cannot safely enter every narrow or unstable space[cite: 16]. **Sanjeevani Rover** is a low-cost, camera-only reconnaissance rover powered by a Raspberry Pi 4 that enters first, detects trapped people completely offline using an on-device YOLOv8-pose AI model, evaluates whether they are conscious (moving) or possibly unconscious (still), transmits CRC-checked alerts with GPS coordinates to a laptop ground station, and plays reassuring voice messages in English and Hindi[cite: 2, 9, 16, 17].

> **Disclaimer:** This is a student prototype and not certified rescue equipment[cite: 9]. Posture and consciousness labels are heuristic estimates based on camera geometry and movement, not a medical diagnosis[cite: 2, 8, 9].

---

## 1. Prototype Gallery

| Front View (Pi Camera & Zebronics Webcam)[cite: 14] | Top View (Dual 2200mAh 80C LiPo & Wiring)[cite: 15] | Side Views (Cardboard Chassis & 4WD)[cite: 12, 13] |
|---|---|---|
| ![Front View](images/rover_front.jpg) | ![Top View](images/rover_top.jpg) | ![Side View](images/rover_side.jpg) |

---

## 2. System Architecture & Files

The rover's four geared motors are driven manually via a FlySky radio remote and motor driver so a human operator remains in the loop at all times; **none of the Python scripts touch the drive motors**[cite: 2, 9, 16, 20].

```text
          ROVER (Raspberry Pi 4B)                                LAPTOP (Windows Ground Station)
 +-----------------------------------------------+       UDP     +-------------------------------+
 | Pi Camera (CSI) --> rover_node.py             | ------------> | ground_station.py             |
 |   YOLOv8n-pose (ONNX Runtime, offline)        |     :5005     |   Coloured alerts + CSV log   |
 |   Posture + movement + 3-of-5 fusion rules    |               |   Google Maps position link   |
 | Neo-6M GPS (UART /dev/serial0) --^            |               +-------------------------------+
 | AMG8833 Thermal (I2C, optional) -^            |
 | Speaker (3.5mm AUX) <-- speaker_alert.py      |       HTTP    +-------------------------------+
 |                                               | ------------> | Web Browser                   |
 | Zebronics USB Webcam --> operator_view.py     |     :8080     |   http://192.168.50.2:8080    |
 +-----------------------------------------------+               +-------------------------------+
        |  (Independent RC Drive Train)
 FlySky Receiver (FS-iA10B) --> Motor Driver --> 4 Geared Motors
```

### Repository File Guide

| File | Runs On | Purpose |
|---|---|---|
| `rover_node.py` | **Pi** | Captures 640x480 @ 15 fps frames from the Pi Camera, runs `yolov8n-pose.onnx`, classifies posture/motion, reads GPS, and transmits UDP alerts[cite: 2, 9]. |
| `ground_station.py` | **Laptop** | Listens on UDP `0.0.0.0:5005`, verifies packet CRCs, displays red/yellow alerts with Google Maps links, and logs every packet to `rescue_telemetry_log.csv`[cite: 5, 9]. |
| `protocol.py` | **Both** | Builds and validates the UTF-8 alert string and computes the 4-hex-digit `CRC-16/CCITT-FALSE` checksum[cite: 8, 9]. |
| `link.py` | **Both** | Standard-library non-blocking UDP socket transmitter (`UdpTx`) and receiver (`UdpRx`)[cite: 6, 9]. |
| `operator_view.py` | **Pi** | Streams the top-mounted USB webcam as MJPEG over HTTP port `8080` with a live corner clock to detect frozen video[cite: 7, 9]. |
| `speaker_alert.py` | **Pi** | Plays bilingual (English and Hindi) voice alerts over the Pi's 3.5 mm headphone jack (`plughw:Headphones`) when a person is found[cite: 11, 17]. |
| `check_all.py` | **Pi** | 1-minute diagnostic check for camera locks, power/throttling (`vcgencmd`), CPU temperature, AI speed, USB webcam, GPS, and laptop link[cite: 4]. |
| `see_detection.py` | **Pi** | Waits 10 seconds, captures one Pi Camera frame, draws red bounding boxes and green/yellow keypoints, and saves `seen.jpg`[cite: 3]. |
| `calibrate_motion.py` | **Pi** | Records 25 seconds of still posture and 25 seconds of limb movement to compute the optimal `MOTION_THRESH` value[cite: 10]. |
| `setup_pi.sh` | **Pi** | One-time setup script for system packages, Python `venv`, I2C, and `/dev/serial0` UART[cite: 9]. |
| `yolov8n-pose.onnx` | **Pi** | Exported YOLOv8n-pose ONNX model for CPU inference via `onnxruntime`[cite: 2, 9]. |
| `rescue_telemetry_log_sample.csv` | **Sample** | Sanitized sample CSV log demonstrating recorded alerts with test GPS coordinates[cite: 5, 8, 16]. |
| `hardware/*.stl` | **CAD** | `motor_cradle.stl` and `motor_clamp.stl` 3D-printable motor mounts designed for the next-stage chassis upgrade. |

---

## 3. How the AI Detection & Alert Protocol Work

### Posture & Movement Rules
1. **17-Keypoint Pose Estimation:** `rover_node.py` runs `yolov8n-pose.onnx` using 3 CPU threads (`ORT_THREADS = 3`)[cite: 2]. In camera-only mode (`THERMAL_MODE = "off"`), a full detection requires a confidence score of at least `0.50` (`CAMONLY_MIN_CONF`) and at least `5` visible keypoints (`CAMONLY_MIN_KEYPOINTS`) above `0.30` visibility (`KP_VIS_THRESH`)[cite: 2, 9]. If a body is partially cut off, a partial detection (`Person detected (partial view)`) is triggered at confidence `>= 0.40` with at least `2` visible keypoints[cite: 2, 8, 11].
2. **3-Second Shape Voting:** Single-frame keypoint fits can jitter, so body geometry (`upright`, `lying`, or `slumped` based on a `60°` lying torso angle or `35°` slump angle) is decided by a majority vote over the last `3.0` seconds (`POSTURE_VOTE_S`)[cite: 2, 9].
3. **5-Second Relative Motion Tracking:** `MotionTracker` normalizes keypoint positions relative to the bounding box center and size over a `5.0`-second window (`MOTION_WINDOW_S`)[cite: 2, 9]. If the average keypoint standard deviation is `>= 0.02` (`MOTION_THRESH`), the person is marked **Conscious**; if below `0.02`, **Unconscious**[cite: 2, 9, 11].
4. **3-of-5 Temporal Fusion:** The main thread checks state snapshots every `0.1` seconds (`FUSION_PERIOD_S`) and only transmits an alert when the newest check is positive and at least **3 of the last 5** checks agree (`CONFIRM_MIN = 3`, `CONFIRM_WINDOW = 5`)[cite: 2, 9, 17].

### Alert Packet Format
Each alert is sent as a single UTF-8 UDP datagram protected by a 4-hex-digit `CRC-16/CCITT-FALSE` checksum after `*`[cite: 8, 9]:

```text
ALERT: Human found! Status: Conscious (Upright/Sitting) | Distance: 2.5 m | Lat: N/A | Lon: N/A | Temp: N/A | Fix: NONE | Seq: 1*587E
```

* **Status Values:** `Conscious (Upright/Sitting)`, `Unconscious (Collapsed/Slumped)`, `Conscious Lying Down`, `Unconscious Lying Down`, or `Person detected (partial view)`.
* **GPS Fix Labels:** `LIVE` (fix age `<= 3.0 s`), `CACHED-<n>s` (older fix), or `NONE` (no fix/indoors; the rover never invents coordinates)[cite: 2, 9, 17].

---

## 4. Hardware Bill of Materials & Wiring

* **Compute:** Raspberry Pi 4 Model B with passive heatsink (maintains 45–50°C under load, down from 62–70°C bare)[cite: 2, 16, 17, 20].
* **Vision:** Raspberry Pi Camera Rev 1.3 (`ov5647` CSI ribbon camera, mounted inverted with `CAM_ROTATE_180 = True`) + Zebronics USB 2.0 Webcam (`/dev/v4l/by-id/*-video-index0`)[cite: 2, 4, 7, 9, 12, 13, 14].
* **Navigation & Audio:** Neo-6M UART GPS module + amplified AUX speaker on the 3.5 mm jack[cite: 9, 11, 16].
* **Drive & Power:** 4 geared motors, motor driver, FlySky transmitter & FS-iA10B receiver, 2x 2200mAh 80C LiPo packs (drive power), and an isolated 5V power bank for the Pi[cite: 9, 11, 15, 16].
* **Chassis:** Rapid-prototyped cardboard enclosure (lightweight, zero-cost, quick modification)[cite: 12, 14, 16, 20].

### GPS Module Wiring (UART `/dev/serial0` @ 9600 baud)[cite: 2, 9]

| GPS Pin | Pi Physical Pin | Pi Signal |
|---|---|---|
| VCC | Pin 1 (3.3V) or Pin 2 (5V) | Power (check board regulator label)[cite: 4, 9] |
| GND | Pin 6 | GND[cite: 4, 9] |
| TX | **Pin 10** | GPIO15 (RXD)[cite: 4, 9] |
| RX | **Pin 8** | GPIO14 (TXD)[cite: 4, 9] |

### Optional AMG8833 Thermal Sensor Wiring (I2C1 @ `0x69` / `0x68`)[cite: 2, 9]

| AMG8833 Pin | Pi Physical Pin | Pi Signal |
|---|---|---|
| VIN | Pin 1 | 3.3V[cite: 9] |
| GND | Pin 9 | GND[cite: 9] |
| SDA | Pin 3 | GPIO2 (SDA)[cite: 9] |
| SCL | Pin 5 | GPIO3 (SCL)[cite: 9] |

---

## 5. Network Setup & Quick-Start Demo Commands

### Static Ethernet Configuration
* **Laptop Ground Station:** IP `192.168.50.1`, Subnet `255.255.255.0` (no default gateway), with UDP port `5005` allowed in Windows Defender Firewall[cite: 9, 11]:
  ```powershell
  netsh advfirewall firewall add rule name="Rescue UDP 5005" dir=in action=allow protocol=UDP localport=5005 profile=any
  ```
* **Raspberry Pi Rover:** IP `192.168.50.2/24` on `eth0` (`ipv4.never-default yes` so Wi-Fi retains internet access)[cite: 9, 11].

### Live Demo Execution Sequence

**Step 1 — Start the Laptop Ground Station (PowerShell Window B):**[cite: 5, 11]
```powershell
cd C:\Users\csc\OneDrive\Desktop\rescue
python ground_station.py
```

**Step 2 — Connect to the Raspberry Pi & Run Health Check (PowerShell Window A):**[cite: 4, 9, 11]
```bash
ssh divyanshu@192.168.50.2
cd ~/rescue && source venv/bin/activate
python3 check_all.py
```

**Step 3 — Start AI Human Detection on the Rover (Pi Window A):**[cite: 2, 9, 11]
```bash
python3 rover_node.py -v
```

**Step 4 — Start Live Operator Driving View (Second Pi SSH Window):**[cite: 7, 9, 11]
```bash
cd ~/rescue && source venv/bin/activate
python3 operator_view.py
```
Open **`[http://192.168.50.2:8080](http://192.168.50.2:8080)`** in Chrome on the laptop to view the live USB webcam stream[cite: 7, 9, 11].

**Useful Diagnostic & Audio Commands on the Pi:**[cite: 3, 10, 11]
```bash
python3 see_detection.py          # Capture 1 frame with AI keypoints to ~/rescue/seen.jpg
python3 calibrate_motion.py       # Calibrate MOTION_THRESH for still vs. moving posture
python3 speaker_alert.py test     # Test English and Hindi voice alerts over the speaker
touch ~/rescue/MUTE               # Mute speaker alerts (remove with: rm ~/rescue/MUTE)
vcgencmd measure_temp             # Check Pi CPU temperature
sudo shutdown now                 # Safely shut down the Pi before unplugging power
```

---

## 6. Design Choices & Roadmap

* **Why Camera-Only?** Keeping a thermal camera optional reduces cost and weight so schools and local rescue teams can build the rover with off-the-shelf parts, while the protocol already reserves the `Temp:` field for a drop-in AMG8833 sensor upgrade[cite: 2, 9, 16, 20].
* **Why ~3 FPS on a CPU?** Running YOLOv8-pose on a Raspberry Pi 4 CPU at ~3 frames per second avoids expensive GPU hardware; combined with the 3-of-5 voting filter, it prioritizes steady, reliable alerts over raw frame rate[cite: 4, 11, 17, 20].
* **Why a Cardboard Chassis?** It enabled rapid, zero-cost iteration of sensor placement and electronics testing while our 3D printer was unavailable[cite: 11, 16, 20].
* **Next Upgrades:**
  1. **3D-Printed Weather-Resistant Body:** Transitioning to our CAD-designed chassis and motor mounts (`motor_cradle.stl`, `motor_clamp.stl`)[cite: 11, 16, 18, 20].
  2. **AMG8833 Thermal Sensor:** Enabling `THERMAL_MODE = "amg8833"` to cross-check body heat and eliminate visual false positives[cite: 2, 9, 18, 20].
  3. **Fiber-Optic / Tethered Link:** Developing a thin physical tether (inspired by fiber-optic drones) to maintain high-bandwidth communication inside deep rubble where radio signals weaken[cite: 18, 20].
