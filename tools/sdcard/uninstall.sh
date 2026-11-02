#!/usr/bin/env bash
# Remove the WSL SD card auto-mount setup. Run as root.
set -euo pipefail
[ "$(id -u)" -eq 0 ] || { echo "run as root" >&2; exit 1; }

if [ -x /usr/local/sbin/sdcard-helper ]; then
    /usr/local/sbin/sdcard-helper release || true
fi
systemctl disable --now sdcard-usbip-attach.service 2>/dev/null || true
systemctl stop 'sdcard-mount@*.service' 2>/dev/null || true
rm -f /etc/systemd/system/sdcard-usbip-attach.service /etc/systemd/system/sdcard-mount@.service
rm -f /etc/udev/rules.d/99-sdcard-automount.rules /etc/sudoers.d/sdcard
rm -f /usr/local/sbin/sdcard-helper /usr/local/bin/sdcard
systemctl daemon-reload
udevadm control --reload
if [ -L /mnt/sdcard ]; then rm -f /mnt/sdcard; fi
echo "removed. /etc/sdcard.conf was left in place."
