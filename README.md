# Sanjeevani Rover

**A low-cost, camera-only search-and-rescue rover.** It finds a person with on-board AI, works out whether they are standing, lying or collapsed and whether they are moving, tells them calmly that they have been seen, and sends a checked alert to a rescue ground station. No internet is needed.

Built by **Divyanshu, Saloni, Anika and Ashutosh** for the **SEVA First Innovation Challenge 2026**.

> **Status: working prototype.** Tested indoors with teammates. It has **not** been tested in a real disaster. See [Limitations](#limitations-stated-honestly).

---

## Why this project

In the first hours after an earthquake, flood or building collapse, rescuers cannot enter every dangerous space. A small rover that goes in first, finds a person and reports back can save time and keep rescuers out of danger. The biggest barrier is cost, so the rover is built from cheap, easy-to-buy parts, and every part can be rebuilt by students.

## Features

| Feature | What it does |
|---|---|
| **Human detection** | YOLOv8n-pose model runs fully on a Raspberry Pi 4 (CPU, ONNX, 480x480, roughly 3 pictures per second). |
| **Posture** | Standing, lying down, or collapsed/slumped, from body key-points. Partial views (for example only part of the body visible) also raise an alert. |
| **Conscious / unconscious guess** | A lying person who moves is reported as *conscious*; one who stays still as *possibly unconscious*. This is based on movement only. |
| **Steady alerts** | An alert is sent only when 3 of the last 5 decisions agree, and at most one alert every 4 seconds. |
| **Checked alerts** | Plain-text alert with a sequence number and CRC-16 check, sent over UDP. The ground station rejects damaged messages. |
| **Ground station** | Laptop program that receives alerts, shows them and logs them to a CSV file. |
| **Operator view** | Live USB-webcam picture in a browser on the laptop. |
| **GPS** | NEO-6M GPS module. The alert says `Fix: NONE` and `Lat/Lon: N/A` when there is no fix; it never invents a position. |
| **Voice message to survivors** | Plays a recorded English then Hindi message through a speaker. It plays when a person is first seen and again for a *new* person, but does not nag the same person. It can be muted. |
| **Self-check** | `check_all.py` reports on camera, GPS, network, power and temperature. |

### What the voice says
> "We can see you. Please stay calm. A rescue alert has been sent to the team."

It states only facts. It does **not** promise that help is arriving soon, because the rover cannot guarantee that.

### Example alert (as received by the ground station)
```
ALERT: Human found! Status: Conscious Lying Down | Distance: N/A | Lat: N/A | Lon: N/A | Temp: N/A | Fix: NONE | Seq: 1*4DF9
```
`Temp: N/A` is normal: this build is camera-only (no thermal sensor). The field is reserved so a thermal sensor can be added later.

---

## How it works

```
  Pi Camera --> YOLOv8n-pose (ONNX) --> posture + movement --> fusion (3 of last 5)
                                                                    |
   GPS (NEO-6M) -----------------------------------------------> alert text + CRC-16
                                                                    |
                                   +--------------------------------+---------------+
                                   |                                                |
                          UDP port 5005 over Ethernet                        speaker voice
                                   |                                          (EN + HI)
                          Ground station (laptop)
                          shows alert, saves CSV log

   USB webcam --> operator_view.py --> browser (http://<Pi>:8080)
```

Driving is done by a normal RC radio. **This repository contains no motor-control code.**

## Hardware

| Part | Used for |
|---|---|
| Raspberry Pi 4 Model B | Runs everything |
| Pi Camera (OV5647) | Human detection |
| USB webcam | Operator view |
| GY-GPS6MV2 (NEO-6M) with antenna | Position |
| Small aux speaker | Voice message to survivors |
| FlySky FS-iA10B receiver, PPM-to-PWM converter, MOSFET boards, 4 yellow 12 V gear motors | Driving by RC radio |
| USB power bank | Powers the Pi (a power bank that can supply a full 5 V / 3 A is recommended) |
| Cardboard chassis | Prototype body (a 3D-printed body is planned) |

## Repository contents

| File | Purpose |
|---|---|
| `rover_node.py` | Main program on the Pi: camera, detection, decision and alert sending |
| `link.py`, `protocol.py` | UDP sender and the alert format with CRC-16 |
| `ground_station.py` | Receiver on the laptop, writes `rescue_telemetry_log.csv` |
| `operator_view.py` | USB webcam streamed to a browser |
| `speaker_alert.py` | Voice message logic (new-person rule, mute) |
| `check_all.py` | Health check (does not touch the motors) |
| `see_detection.py`, `calibrate_motion.py` | Tools for checking detection and tuning the motion threshold |
| `setup_pi.sh` | Installs the Pi software dependencies |
| `audio/` | Recorded voice messages (`found_en.wav`, `found_hi.wav`) |
| `docs/` | Demo commands and photos |

---

## Setup

**Network:** a direct Ethernet cable between the Pi and the laptop with fixed addresses: laptop `192.168.50.1`, Pi `192.168.50.2`.

**Pi:** copy the project folder to `~/rescue`, then run `setup_pi.sh` to install the dependencies and create the `venv` environment. Put a YOLOv8n-pose ONNX model named `yolov8n-pose.onnx` in the same folder.

> **Model file is not included.** YOLOv8 is made by Ultralytics and is licensed under AGPL-3.0. Download or export `yolov8n-pose` yourself from [Ultralytics](https://github.com/ultralytics/ultralytics) (the node expects a 480x480 input) and respect their licence.

**Laptop (Windows):** install Python, then allow the alerts through the firewall once (PowerShell as Administrator):
```powershell
New-NetFirewallRule -DisplayName "Rover UDP 5005" -Direction Inbound -Protocol UDP -LocalPort 5005 -Action Allow
```

## Run the demo

Use two terminal windows. **Start the ground station first.**

**1. Laptop, window B: receiver**
```powershell
cd C:\path\to\rescue
python ground_station.py
```

**2. Laptop, window A: connect to the Pi**
```powershell
ssh <pi-user>@192.168.50.2
```

**3. Pi: start the rover**
```bash
cd ~/rescue && source venv/bin/activate
python3 rover_node.py
```
A person stands 2 to 3 metres from the camera. Alerts appear on the laptop and the voice plays on the Pi.

Useful extras (stop the rover with Ctrl+C first):

| Command (on the Pi) | What it does |
|---|---|
| `python3 check_all.py` | Health check |
| `python3 rover_node.py --dry-run -v` | Prints alerts, sends nothing (safe test) |
| `python3 operator_view.py` | USB webcam view at `http://<Pi address>:8080` |
| `python3 speaker_alert.py test` | Plays the voice message once |
| `touch ~/rescue/MUTE` / `rm ~/rescue/MUTE` | Mute / unmute the voice |
| `vcgencmd measure_temp` | Pi temperature |

See [`docs/Rover_Demo_Commands.pdf`](docs/Rover_Demo_Commands.pdf) for the full step-by-step list.

---

## Limitations (stated honestly)

We would rather say what does not work yet than over-claim.

- **Camera only.** No thermal sensor, so no body-heat check. Detection depends on lighting.
- **About 3 pictures per second** on the Pi 4 CPU. We chose reliable, steady alerts over raw speed.
- **Consciousness is a guess from movement only.** It is a hint for rescuers, not a medical judgement.
- **No accuracy figure yet.** Tested indoors with teammates; we have not measured detection accuracy.
- **Not tested in a real disaster.**
- **The Pi runs hot** (about 62 to 70 degrees C under load). A heat sink and fan are being added.
- **GPS needs open sky.** Indoors the alert says `Fix: NONE`. Outdoor testing is still to be done.
- **The alert link is a wired Ethernet cable for now.** The alert is plain network data, so Wi-Fi or a long-range radio link is the next step.
- **Driven by a human with an RC radio.** It is not autonomous, on purpose: a human stays in control.
- **The voice is one-way.** The rover cannot hear the survivor yet. A microphone is a possible upgrade.
- **Prototype body.** Cardboard chassis; a 3D-printed body is designed for the next version.
- **Safety.** This is a student prototype. It is not certified equipment and must never replace trained rescuers.

## Roadmap

1. Heat sink and fan for the Pi
2. Outdoor GPS testing
3. Wi-Fi / long-range radio link for alerts
4. Thermal sensor (the `Temp` field is already reserved in the alert)
5. 3D-printed, weather-resistant body
6. Microphone so survivors can be heard as well as spoken to
7. Measure detection accuracy with a proper test set

# Sanjeevani Rover — AI Search-and-Rescue Recon Rover[cite: 9, 16]

**SEVA First Innovation Challenge 2026** | **Indorama Public School**[cite: 16]  
**Team:** Divyanshu, Saloni, Anika, and Ashutosh[cite: 16]  
**Readiness Level:** TRL 5 (Prototype tested in an environment close to real use)[cite: 20]

When an earthquake, flood, or building collapse traps people, the first hours decide who lives, and human rescuers cannot safely enter every narrow or unstable space[cite: 16]. **Sanjeevani Rover** is a low-cost, camera-only reconnaissance rover powered by a Raspberry Pi 4 that enters first, detects trapped people completely offline using an on-device YOLOv8-pose AI model, evaluates whether they are conscious (moving) or possibly unconscious (still), transmits CRC-checked alerts with GPS coordinates to a laptop ground station, and plays reassuring voice messages in English and Hindi[cite: 2, 9, 16, 17].

> **Disclaimer:** This is a student prototype and not certified rescue equipment. Posture and consciousness labels are heuristic estimates based on camera geometry and movement, not a medical diagnosis[cite: 2, 8, 9].

---

## 1. Prototype Gallery

| Front View (Pi Camera & Zebronics Webcam)[cite: 14] | Top View (Dual 2200mAh 80C LiPo & Wiring)[cite: 15] | Side Views (Cardboard Chassis & 4WD)[cite: 12, 13] |
|---|---|---|
| ![Front View](images/rover_front.jpg) | ![Top View](images/rover_top.jpg) | ![Side View](images/rover_side.jpg) |

---

## 2. System Architecture & Files

The rover's four geared motors are driven manually via a FlySky radio remote and motor driver so a human operator remains in the loop at all times; **none of the Python scripts touch the drive motors**.

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
 | Zebronics USB Webcam --> operator_view.py     |     :8080     |   [http://192.168.50.2:8080](http://192.168.50.2:8080)    |
 +-----------------------------------------------+               +-------------------------------+
        |  (Independent RC Drive Train)
 FlySky Receiver (FS-iA10B) --> Motor Driver --> 4 Geared Motors
```

## 3. How the AI Detection & Alert Protocol Work

### Posture & Movement Rules
1. **17-Keypoint Pose Estimation:** `rover_node.py` runs `yolov8n-pose.onnx` using 3 CPU threads (`ORT_THREADS = 3`)[cite: 2]. In camera-only mode (`THERMAL_MODE = "off"`), a full detection requires a confidence score of at least `0.50` (`CAMONLY_MIN_CONF`) and at least `5` visible keypoints (`CAMONLY_MIN_KEYPOINTS`) above `0.30` visibility (`KP_VIS_THRESH`)[cite: 2, 9]. If a body is partially cut off, a partial detection (`Person detected (partial view)`) is triggered at confidence `>= 0.40` with at least `2` visible keypoints[cite: 2, 8, 11].
2. **3-Second Shape Voting:** Single-frame keypoint fits can jitter, so body geometry (`upright`, `lying`, or `slumped` based on a `60°` lying torso angle or `35°` slump angle) is decided by a majority vote over the last `3.0` seconds (`POSTURE_VOTE_S`)[cite: 2, 9].
3. **5-Second Relative Motion Tracking:** `MotionTracker` normalizes keypoint positions relative to the bounding box center and size over a `5.0`-second window (`MOTION_WINDOW_S`)[cite: 2, 9]. If the average keypoint standard deviation is `>= 0.02` (`MOTION_THRESH`), the person is marked **Conscious**; if below `0.02`, **Unconscious**[cite: 2, 9, 11].
4. **3-of-5 Temporal Fusion:** The main thread checks state snapshots every `0.1` seconds (`FUSION_PERIOD_S`) and only transmits an alert when the newest check is positive and at least **3 of the last 5** checks agree (`CONFIRM_MIN = 3`, `CONFIRM_WINDOW = 5`)[cite: 2, 9, 17].

### Alert Packet Format
Each alert is sent as a single UTF-8 UDP datagram protected by a 4-hex-digit `CRC-16/CCITT-FALSE` checksum after `*`[cite: 8, 9]:

```text
ALERT: Human found! Status: Conscious (Upright/Sitting) | Distance: 2.1 m | Lat: N/A | Lon: N/A | Temp: N/A | Fix: NONE | Seq: 1*587E
```

* **Status Values:** `Conscious (Upright/Sitting)`, `Unconscious (Collapsed/Slumped)`, `Conscious Lying Down`, `Unconscious Lying Down`, or `Person detected (partial view)`[cite: 8].
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

  # Step 1: Go to your local project folder and create subfolders[cite: 11]
cd C:\Users\csc\OneDrive\Desktop\rescue
mkdir -Force images, hardware

# Step 2: (Optional) Pull any remaining scripts from the Pi if it is powered on and connected via Ethernet[cite: 3, 4, 9, 10, 11]
# Uncomment the line below if those files are not yet in C:\Users\csc\OneDrive\Desktop\rescue:
# scp divyanshu@192.168.50.2:~/rescue/calibrate_motion.py divyanshu@192.168.50.2:~/rescue/see_detection.py divyanshu@192.168.50.2:~/rescue/check_all.py divyanshu@192.168.50.2:~/rescue/speaker_alert.py divyanshu@192.168.50.2:~/rescue/setup_pi.sh .

# Step 3: Move any STL files into hardware/ if they are in the main folder[cite: 11]
if (Test-Path motor_cradle.stl) { Move-Item -Force motor_cradle.stl hardware\ }
if (Test-Path motor_clamp.stl)  { Move-Item -Force motor_clamp.stl hardware\ }

# Step 4: Automatically strip any tags from README.md so it is 100% clean on GitHub
if (Test-Path README.md) {
    (Get-Content README.md -Raw) -replace '\s*\[cite:\s*[\d,\s]+\]', '' | Set-Content -Path README.md -Encoding UTF8
}

# Step 5: Create .gitignore to protect your real GPS logs and temporary files[cite: 2, 5, 9, 10, 11, 16]
@"
__pycache__/
*.pyc
venv/
*.bak
*.1
calib_data.json
rover_events.jsonl
rescue_telemetry_log.csv
webcam_test.py
check.jpg
webcam.jpg
claude full conversation.txt
"@ | Set-Content -Path .gitignore -Encoding UTF8

# Step 6: Initialize Git, pull existing repo history, stage files, commit, and push[cite: 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
git init
git remote remove origin 2>$null
git remote add origin [https://github.com/divyanshu-md/sanjeevani-rover.git](https://github.com/divyanshu-md/sanjeevani-rover.git)
git branch -M main
git pull origin main --allow-unrelated-histories

git add README.md .gitignore
foreach ($f in @("protocol.py","link.py","ground_station.py","rover_node.py","operator_view.py","check_all.py","see_detection.py","calibrate_motion.py","speaker_alert.py","setup_pi.sh","yolov8n-pose.onnx","images","hardware")) {
    if (Test-Path $f) { git add $f }
}

git status
git commit -m "Add Sanjeevani Rover AI detection, ground station, diagnostics, and documentation"
git push -u origin main


## Credits

- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) pose model (AGPL-3.0)
- [Picamera2 / libcamera](https://github.com/raspberrypi/picamera2) for the Pi Camera
- [eSpeak NG](https://github.com/espeak-ng/espeak-ng) as a fallback robot voice

## Licence

The code in this repository is released under the [MIT licence](LICENSE). Third-party components keep their own licences.
