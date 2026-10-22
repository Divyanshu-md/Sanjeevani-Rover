#!/usr/bin/env bash
# =============================================================================
# setup_pi.sh  -  one-time setup of the ROVER side (run this ON THE PI)
# =============================================================================
# WHAT IT DOES (it prints each step, and it is safe to run again):
#   1. installs the system packages (camera, OpenCV, serial, I2C tools, ...)
#   2. creates the Python environment  <this folder>/venv  (if missing)
#      and installs the extra Python packages inside it
#   3. switches on I2C  (needed later for the AMG8833 thermal sensor)
#   4. switches on the hardware serial port for the GPS
#        - enable_uart=1 and dtoverlay=disable-bt in config.txt (only if missing)
#        - the serial LOGIN console is switched off (it would steal the port)
#   5. adds your user to the groups dialout, i2c and video
#   6. asks before rebooting (a reboot is needed for steps 3-5 to take effect)
#
# WHAT IT DOES NOT DO
#   * It does not touch your network settings (Wi-Fi / Ethernet IP addresses).
#   * It does not touch the motors or the FlySky / Cytron wiring.
#   * It does not download the AI model.  You copy yolov8n-pose.onnx yourself
#     (see README.md, section "Export the AI model").
#
# HOW TO RUN (as your normal user, NOT as root, NOT with sudo in front):
#     cd ~/rescue
#     bash setup_pi.sh
# It will ask for your password when it needs sudo.  That is normal.
# =============================================================================

set -u
set -o pipefail

# --- where things are --------------------------------------------------------
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENV_DIR="$PROJECT_DIR/venv"

# Bookworm keeps the file in /boot/firmware, older systems in /boot.
# (BOOT_CONFIG can be set from outside; the self-test of this script uses that.)
if [ -z "${BOOT_CONFIG:-}" ]; then
    if [ -f /boot/firmware/config.txt ]; then
        BOOT_CONFIG=/boot/firmware/config.txt
    else
        BOOT_CONFIG=/boot/config.txt
    fi
fi

WARNINGS=0

say()  { printf '\n==> %s\n' "$*"; }
ok()   { printf '    OK: %s\n' "$*"; }
warn() { printf '    WARNING: %s\n' "$*"; WARNINGS=$((WARNINGS + 1)); }

# ensure_line FILE LINE : append LINE to FILE only if it is not already there
# (a line that is commented out with # does NOT count as "there").
ensure_line() {
    local file="$1" line="$2"
    if grep -qxF "$line" "$file" 2>/dev/null; then
        ok "already in $file:  $line"
        return 0
    fi
    # sudo tee -a works for the real /boot file; for a test file it also works.
    if printf '%s\n' "$line" | sudo tee -a "$file" >/dev/null; then
        ok "added to $file:  $line"
        CONFIG_CHANGED=1
    else
        warn "could not write to $file.  Add this line by hand:  $line"
    fi
}

CONFIG_CHANGED=0

# --- 0. safety checks ---------------------------------------------------------
if [ "$(id -u)" -eq 0 ]; then
    echo "Please run this as your normal user (without sudo):  bash setup_pi.sh"
    echo "(It uses sudo by itself only where needed.)"
    exit 1
fi

if ! command -v sudo >/dev/null 2>&1; then
    echo "sudo was not found.  This script is meant for Raspberry Pi OS."
    exit 1
fi

echo "Rover setup. Project folder: $PROJECT_DIR"
echo "User: $USER      Boot config: $BOOT_CONFIG"

# --- 1. system packages ---------------------------------------------------------
say "1/6  Installing system packages (this can take a few minutes)"
if sudo apt-get update; then
    ok "package list updated"
else
    warn "apt-get update failed (no internet?).  Trying to install anyway."
fi

if sudo apt-get install -y python3-venv python3-pip python3-opencv python3-picamera2 \
        python3-serial python3-numpy i2c-tools v4l-utils; then
    ok "system packages installed"
else
    warn "some system packages did not install.  Check the Pi has internet, then run this script again."
fi

# --- 2. Python environment ---------------------------------------------------------
say "2/6  Python environment in $VENV_DIR"
if [ -x "$VENV_DIR/bin/python3" ]; then
    ok "venv already exists"
else
    # --system-site-packages is REQUIRED: picamera2 and OpenCV come from apt.
    if python3 -m venv --system-site-packages "$VENV_DIR"; then
        ok "venv created"
    else
        warn "could not create the venv"
    fi
fi

if [ -x "$VENV_DIR/bin/pip" ]; then
    if "$VENV_DIR/bin/pip" install --upgrade pip \
        && "$VENV_DIR/bin/pip" install adafruit-circuitpython-amg88xx pynmea2 onnxruntime; then
        ok "pip packages installed (adafruit-circuitpython-amg88xx, pynmea2, onnxruntime)"
    else
        warn "pip install failed.  Check internet and run this script again."
    fi
else
    warn "venv has no pip, skipping pip packages"
fi

# --- 3. I2C -------------------------------------------------------------------------
say "3/6  Switching on I2C (for the thermal sensor, later)"
if command -v raspi-config >/dev/null 2>&1; then
    # In raspi-config's non-interactive mode, 0 means "enable".
    if sudo raspi-config nonint do_i2c 0; then
        ok "I2C enabled"
    else
        warn "raspi-config could not enable I2C.  By hand: sudo raspi-config -> Interface Options -> I2C -> Yes"
    fi
else
    warn "raspi-config not found.  Add the line  dtparam=i2c_arm=on  to $BOOT_CONFIG by hand."
fi

# --- 4. serial port for the GPS ---------------------------------------------------------
say "4/6  Preparing the serial port for the GPS"
if command -v raspi-config >/dev/null 2>&1; then
    # do_serial_cons 1 = login console OFF ;  do_serial_hw 0 = serial hardware ON
    if sudo raspi-config nonint do_serial_cons 1; then
        ok "serial login console switched off"
    else
        warn "could not switch the serial console off.  By hand: sudo raspi-config -> Interface Options -> Serial Port -> login shell: NO, hardware: YES"
    fi
    if sudo raspi-config nonint do_serial_hw 0; then
        ok "serial hardware switched on"
    else
        warn "could not switch the serial hardware on.  See the by-hand route above."
    fi
else
    warn "raspi-config not found.  Do the serial steps by hand (see README.md)."
fi

if [ -f "$BOOT_CONFIG" ]; then
    if [ "$CONFIG_CHANGED" -eq 0 ] && ! grep -qxF "enable_uart=1" "$BOOT_CONFIG"; then
        sudo cp "$BOOT_CONFIG" "$BOOT_CONFIG.bak-rover" 2>/dev/null \
            && ok "backup of the config saved as $BOOT_CONFIG.bak-rover"
    fi
    ensure_line "$BOOT_CONFIG" "enable_uart=1"
    # disable-bt gives the good serial port (/dev/serial0 = ttyAMA0) to the GPS.
    # Side effect: the Pi's own Bluetooth stops working.  Wi-Fi is not affected.
    ensure_line "$BOOT_CONFIG" "dtoverlay=disable-bt"
else
    warn "$BOOT_CONFIG not found.  Add  enable_uart=1  and  dtoverlay=disable-bt  by hand."
fi

# --- 5. groups -------------------------------------------------------------------------------
say "5/6  Giving user '$USER' access to serial, I2C and camera devices"
for grp in dialout i2c video; do
    if getent group "$grp" >/dev/null 2>&1; then
        if id -nG "$USER" | tr ' ' '\n' | grep -qx "$grp"; then
            ok "already in group $grp"
        elif sudo usermod -aG "$grp" "$USER"; then
            ok "added to group $grp (takes effect after reboot / new login)"
            CONFIG_CHANGED=1
        else
            warn "could not add you to group $grp"
        fi
    else
        warn "group $grp does not exist on this system (skipped)"
    fi
done

# --- quick self-check ------------------------------------------------------------------------------
say "Self-check (what is installed now)"
if [ -x "$VENV_DIR/bin/python3" ]; then
    "$VENV_DIR/bin/python3" - <<'PYEOF'
import importlib

def check(name):
    try:
        importlib.import_module(name)
        print("    OK  :", name)
    except Exception as exc:
        print("    MISSING:", name, "->", exc)

for module in ("numpy", "cv2", "picamera2", "serial", "pynmea2", "onnxruntime"):
    check(module)
PYEOF
fi

if [ -f "$PROJECT_DIR/yolov8n-pose.onnx" ]; then
    ok "AI model file found: yolov8n-pose.onnx ($(du -h "$PROJECT_DIR/yolov8n-pose.onnx" | cut -f1))"
else
    warn "AI model file yolov8n-pose.onnx is NOT in $PROJECT_DIR yet (see README.md, 'Export the AI model')."
fi

# --- 6. reboot ------------------------------------------------------------------------------------------
say "6/6  Finished"
if [ "$WARNINGS" -gt 0 ]; then
    echo "    Finished with $WARNINGS warning(s).  Read the WARNING lines above."
else
    echo "    No warnings."
fi

if [ "$CONFIG_CHANGED" -eq 1 ]; then
    echo
    echo "A reboot is needed so the serial / I2C / group changes take effect."
    printf 'Reboot now? [y/N] '
    read -r answer
    case "$answer" in
        y|Y|yes|YES)
            echo "Rebooting in 3 seconds... (your ssh window will close; wait ~1 minute, then ssh in again)"
            sleep 3
            sudo reboot
            ;;
        *)
            echo "OK, not rebooting.  Please run  sudo reboot  yourself before using the GPS."
            ;;
    esac
else
    echo "No reboot is needed."
fi
