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

## Credits

- [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics) pose model (AGPL-3.0)
- [Picamera2 / libcamera](https://github.com/raspberrypi/picamera2) for the Pi Camera
- [eSpeak NG](https://github.com/espeak-ng/espeak-ng) as a fallback robot voice

## Licence

The code in this repository is released under the [MIT licence](LICENSE). Third-party components keep their own licences.
