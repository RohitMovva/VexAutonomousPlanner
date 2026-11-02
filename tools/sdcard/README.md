# SD card auto-mount for the path planner (WSL)

The planner runs in WSL, but the V5 brain reads trajectories from an SD card
that Windows owns. This setup makes the card show up inside WSL at
`/mnt/sdcard` by itself whenever it is inserted, so the planner can write
trajectories straight to `/mnt/sdcard/routes` (the brain's `/usd/routes`).

```
insert card ──▶ usbipd (Windows) ──▶ sdcard-usbip-attach.service keeps the
                                      reader attached to WSL (usbip client)
                                            │
                              kernel sees /dev/sdX1 (FAT32)
                                            │
                       udev: 99-sdcard-automount.rules ──▶ sdcard-mount@sdX1.service
                                            │             mounts at /mnt/wsl/sdcard
                                            │             (/mnt/sdcard is a symlink to it)
                       planner: status bar shows "SD card: /mnt/sdcard/routes",
                       every save also writes <route>.txt there, and anything
                       saved while the card was out is copied over on insert
pull card  ──▶ device disappears ──▶ systemd stops the mount unit (unmount)
```

## One-time setup

1. **Share the reader with WSL** (admin PowerShell, once per reader; it is
   remembered across reboots). Find the reader's BUSID with `usbipd list`:

   ```powershell
   usbipd bind --busid 1-3
   ```

   The built-in reader (Genesys Logic `05e3:0749`, bus 1-3) is already bound.

   More than one reader is fine: list every id in `USB_HARDWARE_ID` in
   `/etc/sdcard.conf` (space separated; the default covers the built-in
   `05e3:0749` and the external `05e3:0764`), and whichever one is plugged in
   and shared gets attached. Swapping readers needs no config change. Re-running
   `install.sh` merges the default ids into an existing conf.

2. **Install the WSL side** (needs root; the sudoers rule it adds is what lets
   the planner eject/mount without a password afterwards):

   ```bash
   sudo tools/sdcard/install.sh
   # or, without a sudo password prompt:
   wsl.exe -u root -- bash tools/sdcard/install.sh --user tihor
   ```

   Then check:

   ```bash
   sdcard status
   ```

The installer is idempotent; re-run it after editing anything in this folder.

## Day to day

- Insert the card. Within a few seconds `sdcard status` shows it mounted and
  the planner's status bar reads `SD card: /mnt/sdcard/routes`.
- Save in the planner (Ctrl+S / autosave). The header folder in `config.yaml`
  is `/mnt/sdcard/routes`, so the trajectory goes straight onto the card.
- If the card is out when you save, the trajectory is parked in
  `route_backups/sd_card_pending/` and moved onto the card the moment it
  appears (File ▸ SD Card ▸ Sync Trajectories, or the "Sync to SD Card"
  button, does the same on demand). No error dialogs for a missing card.
- Prefer keeping trajectories on disk too? Point the header folder at a local
  folder instead: every save is then mirrored to the card, and anything newer
  locally is copied over when the card appears.
- The folder pickers in Settings list the card (`sdcard` and `routes`) in
  their sidebar while it is mounted; with the card out they say so instead of
  opening an empty dialog.
- Before pulling the card: File ▸ SD Card ▸ Eject (Ctrl+Shift+E), the "Eject SD
  Card" button, or `sdcard eject`. Writes are flushed promptly (`flush` mount
  option), so a pull right after a save is fine too, but eject is the safe way.
- Need the card in Windows Explorer for a bit? `sdcard release` hands the reader
  back to Windows and pauses auto-attach; `sdcard attach` (or File ▸ SD Card ▸
  Connect) takes it back.

While WSL is running and auto-attach is on, Windows does not see the reader.
That is the trade-off for hands-off mounting; `sdcard release` is the escape
hatch.

## Why the mount lives under /mnt/wsl

The card is mounted at `/mnt/wsl/sdcard`, and `/mnt/sdcard` is a symlink to it.
That indirection is not cosmetic:

WSL2 runs your login shells in a **different mount namespace from PID 1**, and
`/` is private (no `shared:` peer group) in both. A `mount(8)` performed inside
a systemd unit therefore lands only in systemd's namespace and is invisible to
your shell, to the planner, and to `import_logs.py`. The symptom is nasty,
because every layer reports success:

```
$ journalctl -u 'sdcard-mount@*'     # "mounted /dev/sdf1 ... at /mnt/sdcard"
$ grep sdcard /proc/1/mounts         # the card is there
$ grep sdcard /proc/self/mounts      # ... and not here
$ ls /mnt/sdcard                     # empty
$ sdcard mount                       # exits 0, prints nothing, changes nothing
```

(that last one because `mount_auto` delegates to `systemctl start
sdcard-mount@sdX1.service`, which is already `active (exited)` with
`RemainAfterExit=yes`, so systemd correctly does nothing.)

`/mnt/wsl` is the one place WSL sets up for this: it is a tmpfs tagged
`shared:1` in *every* namespace, so mounts underneath it propagate to all of
them. Keep `MOUNT_POINT` under `/mnt/wsl`.

Two consequences worth remembering:

- `/mnt/wsl` is a tmpfs, so `/mnt/wsl/sdcard` disappears on every WSL restart.
  `sdcard-helper` recreates it (`ensure-mount-point`) before each mount rather
  than relying on the installer having made it once.
- `os.path.ismount()` returns `False` for a symlink without resolving it, so
  anything checking `/mnt/sdcard` must `realpath` it first. `SdCard.is_mounted`
  in `src/utilities/sd_card.py` does.

To verify propagation is working:

```bash
grep sdcard /proc/1/mounts /proc/self/mounts   # both should list the card
```

## Commands

| Command          | What it does                                                    |
|------------------|-----------------------------------------------------------------|
| `sdcard status`  | Reader attached? Card mounted? Label, free space, route count.  |
| `sdcard eject`   | Flush + unmount so the card can be pulled.                      |
| `sdcard mount`   | Mount a card that is inserted but not mounted (after an eject). |
| `sdcard release` | Unmount, detach, stop auto-attach: the reader goes to Windows.  |
| `sdcard attach`  | Resume auto-attach and wait for the card to mount.              |
| `sdcard log [N]` | Last N journal lines from the helper and its units.             |

If the card is inserted and attached but `/mnt/sdcard` is empty while
`/proc/1/mounts` lists it, the mount landed in the wrong namespace: check that
`MOUNT_POINT` in `/etc/sdcard.conf` is under `/mnt/wsl`, then re-run
`sudo tools/sdcard/install.sh` (it migrates an older `/mnt/sdcard` setting and
ejects first). To recover a single session without re-installing:

```bash
sudo -n /usr/local/sbin/sdcard-helper mount-dev /dev/sdf1
```

## Files

| Installed as                                    | From                          | Role |
|-------------------------------------------------|-------------------------------|------|
| `/usr/local/sbin/sdcard-helper`                 | `sdcard-helper`               | Root-side logic: attach loop, mount/unmount, eject, release. |
| `/usr/local/bin/sdcard`                         | `sdcard`                      | User CLI; calls the helper via `sudo -n`. |
| `/etc/sdcard.conf`                              | `sdcard.conf`                 | Reader hardware id, mount point, compat symlink, owner, host IP. |
| `/etc/systemd/system/sdcard-usbip-attach.service` | same name                   | Keeps the reader attached to WSL (enabled at boot). |
| `/etc/systemd/system/sdcard-mount@.service`     | same name                     | Per-device mount unit, bound to the block device. |
| `/etc/udev/rules.d/99-sdcard-automount.rules`   | same name                     | Starts the mount unit for USB FAT32/exFAT partitions. |
| `/etc/sudoers.d/sdcard`                         | generated by `install.sh`     | `NOPASSWD` for the helper only. |

The planner side is `src/utilities/sd_card.py` plus the SD card bits in
`gui/gui_manager.py` and `gui/settings_widget.py`, configured by the `sd_card`
section of `config.yaml` (`mount_point`, `routes_folder`, `pending_folder`,
`auto_sync`) and `files.header_folder` (where trajectories are written).

## Troubleshooting

- **`sdcard status` says "not attached"**: on Windows run `usbipd list`. The
  reader must show `Shared` (bind it if it says `Not shared`) and it must be
  plugged in. `sdcard log` shows what the attach loop is doing.
- **Attached but "card: not mounted"**: is a card inserted? `lsblk -f` should
  show a `vfat`/`exfat` partition; `sdcard mount` mounts it by hand and prints
  why it cannot. Only FAT32/exFAT are mounted, which is what the V5 brain wants.
- **Card inserted, still "not mounted", `lsblk` shows only a 0 B `sde`**: the
  reader re-plugs itself on every card change, and a re-attach that lands
  while Windows is still re-initialising it leaves a half-alive link whose
  disk probe hangs. The attach loop now waits `REATTACH_GRACE` seconds after
  a drop and re-attaches by itself when the disk does not appear within
  `STORAGE_TIMEOUT` seconds (both in `/etc/sdcard.conf`). To force it by hand:
  `sdcard release` then `sdcard attach`.
- **Cannot reach usbipd**: the helper tries the default gateway (NAT
  networking) then `127.0.0.1` (mirrored networking). Set `HOST_IP` in
  `/etc/sdcard.conf` and `sudo systemctl restart sdcard-usbip-attach`.
- **Planner says "The SD card helper is not installed"**: step 2 above was not
  run (or was run on a different distro).
- **The reader moved to another USB port**: `bind` is remembered by the
  device's serial, so normally nothing to do; if `usbipd list` shows it as
  `Not shared` on the new port, bind the new BUSID.
- **After `wsl --shutdown`**: the service starts with WSL and re-attaches;
  nothing to do.

## Uninstall

```bash
sudo tools/sdcard/uninstall.sh      # releases the reader to Windows first
```

`usbipd unbind --busid 1-3` (admin PowerShell) undoes the Windows-side share.
