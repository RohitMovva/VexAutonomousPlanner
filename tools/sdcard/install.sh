#!/usr/bin/env bash
# Install (or update) the WSL SD card auto-mount setup. Idempotent. Run as root:
#   wsl.exe -u root -- bash tools/sdcard/install.sh --user tihor
#   sudo tools/sdcard/install.sh
set -euo pipefail

HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
USER_NAME=${SUDO_USER:-}
HW_ID=""

while [ $# -gt 0 ]; do
    case "$1" in
        --user) USER_NAME=$2; shift 2 ;;
        --hardware-id) HW_ID=$2; shift 2 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done

[ "$(id -u)" -eq 0 ] || { echo "run as root" >&2; exit 1; }
if [ -z "$USER_NAME" ]; then
    USER_NAME=$(awk '/^\[user\]/ {u=1; next} /^\[/ {u=0} u && /^default/ {sub(/.*=[ \t]*/, ""); print; exit}' /etc/wsl.conf 2>/dev/null || true)
fi
[ -n "$USER_NAME" ] && id "$USER_NAME" >/dev/null 2>&1 || { echo "cannot determine the planner user; pass --user <name>" >&2; exit 1; }

install -m 0755 -o root -g root "$HERE/sdcard-helper" /usr/local/sbin/sdcard-helper
install -m 0755 -o root -g root "$HERE/sdcard" /usr/local/bin/sdcard
install -m 0644 -o root -g root "$HERE/sdcard-mount@.service" /etc/systemd/system/sdcard-mount@.service
install -m 0644 -o root -g root "$HERE/sdcard-usbip-attach.service" /etc/systemd/system/sdcard-usbip-attach.service
install -m 0644 -o root -g root "$HERE/99-sdcard-automount.rules" /etc/udev/rules.d/99-sdcard-automount.rules

if [ ! -e /etc/sdcard.conf ]; then
    sed -e "s/^MOUNT_USER=.*/MOUNT_USER=\"$USER_NAME\"/" "$HERE/sdcard.conf" > /etc/sdcard.conf
    chmod 0644 /etc/sdcard.conf
    echo "wrote /etc/sdcard.conf (MOUNT_USER=$USER_NAME)"
else
    echo "kept existing /etc/sdcard.conf"
fi
if [ -n "$HW_ID" ]; then
    sed -i "s/^USB_HARDWARE_ID=.*/USB_HARDWARE_ID=\"$HW_ID\"/" /etc/sdcard.conf
else
    # Merge the shipped reader ids into a kept conf, so an install that only knew one
    # reader picks up the others without losing any it added itself.
    ids=$(. /etc/sdcard.conf; echo "${USB_HARDWARE_ID:-}")
    shipped=$(. "$HERE/sdcard.conf"; echo "$USB_HARDWARE_ID")
    merged=$(printf '%s\n' ${ids//,/ } $shipped | tr 'A-F' 'a-f' | awk 'NF && !seen[$0]++' | paste -sd' ')
    if [ "$merged" != "$ids" ]; then
        sed -i "s/^USB_HARDWARE_ID=.*/USB_HARDWARE_ID=\"$merged\"/" /etc/sdcard.conf
        echo "reader ids in /etc/sdcard.conf: $merged"
    fi
fi

# Let the planner user drive the helper (and only the helper) without a password.
printf '%s ALL=(root) NOPASSWD: /usr/local/sbin/sdcard-helper\n' "$USER_NAME" > /etc/sudoers.d/sdcard.tmp
visudo -cf /etc/sudoers.d/sdcard.tmp >/dev/null
install -m 0440 -o root -g root /etc/sudoers.d/sdcard.tmp /etc/sudoers.d/sdcard
rm -f /etc/sudoers.d/sdcard.tmp

# Creates MOUNT_POINT (under /mnt/wsl, so it must propagate across WSL's mount
# namespaces) and the /mnt/sdcard compatibility symlink. Existing installs that
# still have MOUNT_POINT=/mnt/sdcard in /etc/sdcard.conf are migrated first.
if grep -q '^MOUNT_POINT="/mnt/sdcard"$' /etc/sdcard.conf; then
    # The old mount can exist in two namespaces at once: the unit's ExecStop
    # only unmounts systemd's copy, so clear this one separately. Both have to
    # go before /mnt/sdcard can become a symlink.
    systemctl stop 'sdcard-mount@*.service' 2>/dev/null || true
    for _ in 1 2 3; do
        mountpoint -q /mnt/sdcard || break
        sync -f /mnt/sdcard 2>/dev/null || true
        umount /mnt/sdcard 2>/dev/null || umount -l /mnt/sdcard 2>/dev/null || break
    done
    sed -i 's|^MOUNT_POINT="/mnt/sdcard"$|MOUNT_POINT="/mnt/wsl/sdcard"|' /etc/sdcard.conf
    grep -q '^COMPAT_LINK=' /etc/sdcard.conf || printf 'COMPAT_LINK="/mnt/sdcard"\n' >> /etc/sdcard.conf
    echo "migrated /etc/sdcard.conf to MOUNT_POINT=/mnt/wsl/sdcard (see README: mount namespaces)"
fi
/usr/local/sbin/sdcard-helper ensure-mount-point

udevadm control --reload
systemctl daemon-reload
systemctl enable sdcard-usbip-attach.service >/dev/null 2>&1
systemctl restart sdcard-usbip-attach.service

echo "installed. Check with: sdcard status"
