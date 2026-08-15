#!/usr/bin/env bash
# Pair a game controller to the Pi.
#
#   1. Hold PS + Create (the small button left of the touchpad) on a DualSense
#      until the light bar flashes fast in two-blink bursts.
#   2. Run this.
#
# bluetoothctl is a shell of its own and does not take a script, so this feeds
# it commands and watches its output. The pairing itself is the same three
# steps you would type: pair, trust, connect. Trust is the one that matters
# afterwards -- without it the pad has to be re-authorised every time the Pi
# reboots, which on a car means every time you switch it on.
set -u

WANT='(?i)(dualsense|dualshock|wireless controller|xbox|controller|gamepad)'
SCAN_SECONDS=25

echo "scanning for ${SCAN_SECONDS}s -- put the controller in pairing mode now"
echo

coproc BT { stdbuf -oL bluetoothctl; }
send() { printf '%s\n' "$1" >&"${BT[1]}"; sleep 0.4; }

send "power on"
send "agent on"
send "default-agent"
send "scan on"

mac=""
name=""
deadline=$(( SECONDS + SCAN_SECONDS ))
while [ $SECONDS -lt $deadline ]; do
    if read -r -t 1 line <&"${BT[0]}"; then
        # [NEW] Device AA:BB:CC:DD:EE:FF DualSense Wireless Controller
        if [[ "$line" =~ Device\ ([0-9A-F:]{17})\ (.+)$ ]]; then
            candidate_mac="${BASH_REMATCH[1]}"
            candidate_name="${BASH_REMATCH[2]%$'\r'}"
            if grep -qP "$WANT" <<<"$candidate_name"; then
                mac="$candidate_mac"
                name="$candidate_name"
                break
            fi
        fi
    fi
done

if [ -z "$mac" ]; then
    send "scan off"
    send "quit"
    echo
    echo "No controller appeared."
    echo "  - a DualSense only advertises while the light bar is flashing fast;"
    echo "    it stops after a couple of minutes, so hold PS + Create again"
    echo "  - if it is currently connected to a PlayStation or a phone it will"
    echo "    not advertise at all"
    exit 1
fi

echo "found: $name  [$mac]"
send "scan off"
send "pair $mac"
sleep 3
send "trust $mac"
sleep 1
send "connect $mac"
sleep 3
send "quit"

echo
sleep 1
if bluetoothctl info "$mac" | grep -q "Connected: yes"; then
    echo "connected."
else
    echo "pairing did not take. Try once more, or remove a stale entry first:"
    echo "    bluetoothctl remove $mac"
    exit 1
fi

echo
echo "input devices now present:"
for dev in /dev/input/event*; do
    n=$(cat "/sys/class/input/$(basename "$dev")/device/name" 2>/dev/null)
    [ -n "$n" ] && echo "    $dev  $n"
done

echo
echo "The robot picks it up on its own within a couple of seconds -- the"
echo "Gamepad line in Diagnostics will show its name. No restart needed."
