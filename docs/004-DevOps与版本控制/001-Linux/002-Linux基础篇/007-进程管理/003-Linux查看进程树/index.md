# Linux查看进程树

```Bash
pstree [选项] # 可以更加直观的来查看进程信息。
```

- `-p`: 显示进程的PID。
- `-u`: 显示进程的所属用户。

示例:

```Shell
[root@hxlinux ~]# pstree -up | grep ssh
           |           |                       |                       |-ssh-agent(3386)
           |-sshd(4435)-+-sshd(4450)---bash(4454)-+-grep(5454)
           |            `-sshd(5363)---bash(5367)---su(5420)---bash(5421,heng_xin)
```

全部:

```Shell
[root@hxlinux ~]# pstree
systemd─┬─ModemManager───2*[{ModemManager}]
        ├─NetworkManager───2*[{NetworkManager}]
        ├─VGAuthService
        ├─abrt-dbus───2*[{abrt-dbus}]
        ├─2*[abrt-watch-log]
        ├─abrtd
        ├─accounts-daemon───2*[{accounts-daemon}]
        ├─alsactl
        ├─at-spi-bus-laun─┬─dbus-daemon───{dbus-daemon}
        │                 └─3*[{at-spi-bus-laun}]
        ├─at-spi2-registr───2*[{at-spi2-registr}]
        ├─atd
        ├─auditd─┬─audispd─┬─sedispatch
        │        │         └─{audispd}
        │        └─{auditd}
        ├─avahi-daemon───avahi-daemon
        ├─bluetoothd
        ├─boltd───2*[{boltd}]
        ├─chronyd
        ├─colord───2*[{colord}]
        ├─crond
        ├─cupsd
        ├─2*[dbus-daemon───{dbus-daemon}]
        ├─dbus-launch
        ├─dconf-service───2*[{dconf-service}]
        ├─dnsmasq───dnsmasq
        ├─evolution-addre─┬─evolution-addre───5*[{evolution-addre}]
        │                 └─4*[{evolution-addre}]
        ├─evolution-calen─┬─evolution-calen───8*[{evolution-calen}]
        │                 └─4*[{evolution-calen}]
        ├─evolution-sourc───3*[{evolution-sourc}]
        ├─firewalld───{firewalld}
        ├─fwupd───4*[{fwupd}]
        ├─gdm─┬─X───17*[{X}]
        │     ├─gdm-session-wor─┬─gnome-session-b─┬─abrt-applet───2*[{abrt-applet}]
        │     │                 │                 ├─gnome-shell─┬─ibus-daemon─┬─ibus-dconf───3*[{ibus+
        │     │                 │                 │             │             ├─ibus-engine-sim───2*[+
        │     │                 │                 │             │             └─2*[{ibus-daemon}]
        │     │                 │                 │             └─56*[{gnome-shell}]
        │     │                 │                 ├─gnome-software───3*[{gnome-software}]
        │     │                 │                 ├─gsd-a11y-settin───3*[{gsd-a11y-settin}]
        │     │                 │                 ├─gsd-account───3*[{gsd-account}]
        │     │                 │                 ├─gsd-clipboard───2*[{gsd-clipboard}]
        │     │                 │                 ├─gsd-color───3*[{gsd-color}]
        │     │                 │                 ├─gsd-datetime───3*[{gsd-datetime}]
        │     │                 │                 ├─gsd-disk-utilit───2*[{gsd-disk-utilit}]
        │     │                 │                 ├─gsd-housekeepin───3*[{gsd-housekeepin}]
        │     │                 │                 ├─gsd-keyboard───3*[{gsd-keyboard}]
        │     │                 │                 ├─gsd-media-keys───3*[{gsd-media-keys}]
        │     │                 │                 ├─gsd-mouse───3*[{gsd-mouse}]
        │     │                 │                 ├─gsd-power───3*[{gsd-power}]
        │     │                 │                 ├─gsd-print-notif───2*[{gsd-print-notif}]
        │     │                 │                 ├─gsd-rfkill───2*[{gsd-rfkill}]
        │     │                 │                 ├─gsd-screensaver───2*[{gsd-screensaver}]
        │     │                 │                 ├─gsd-sharing───3*[{gsd-sharing}]
        │     │                 │                 ├─gsd-smartcard───4*[{gsd-smartcard}]
        │     │                 │                 ├─gsd-sound───3*[{gsd-sound}]
        │     │                 │                 ├─gsd-wacom───2*[{gsd-wacom}]
        │     │                 │                 ├─gsd-xsettings───3*[{gsd-xsettings}]
        │     │                 │                 ├─nautilus-deskto───3*[{nautilus-deskto}]
        │     │                 │                 ├─orca───4*[{orca}]
        │     │                 │                 ├─seapplet
        │     │                 │                 ├─ssh-agent
        │     │                 │                 ├─tracker-extract───13*[{tracker-extract}]
        │     │                 │                 ├─tracker-miner-a───3*[{tracker-miner-a}]
        │     │                 │                 ├─tracker-miner-f───3*[{tracker-miner-f}]
        │     │                 │                 ├─tracker-miner-u───3*[{tracker-miner-u}]
        │     │                 │                 └─3*[{gnome-session-b}]
        │     │                 └─2*[{gdm-session-wor}]
        │     └─3*[{gdm}]
        ├─geoclue───2*[{geoclue}]
        ├─gnome-keyring-d───3*[{gnome-keyring-d}]
        ├─gnome-shell-cal───5*[{gnome-shell-cal}]
        ├─gnome-terminal-─┬─bash
        │                 ├─gnome-pty-helpe
        │                 └─3*[{gnome-terminal-}]
        ├─goa-daemon───3*[{goa-daemon}]
        ├─goa-identity-se───3*[{goa-identity-se}]
        ├─gsd-printer───2*[{gsd-printer}]
        ├─gssproxy───5*[{gssproxy}]
        ├─gvfs-afc-volume───3*[{gvfs-afc-volume}]
        ├─gvfs-goa-volume───2*[{gvfs-goa-volume}]
        ├─gvfs-gphoto2-vo───2*[{gvfs-gphoto2-vo}]
        ├─gvfs-mtp-volume───2*[{gvfs-mtp-volume}]
        ├─gvfs-udisks2-vo───2*[{gvfs-udisks2-vo}]
        ├─gvfsd─┬─gvfsd-burn───2*[{gvfsd-burn}]
        │       ├─gvfsd-trash───2*[{gvfsd-trash}]
        │       └─2*[{gvfsd}]
        ├─gvfsd-fuse───5*[{gvfsd-fuse}]
        ├─gvfsd-metadata───2*[{gvfsd-metadata}]
        ├─ibus-portal───2*[{ibus-portal}]
        ├─ibus-x11───2*[{ibus-x11}]
        ├─imsettings-daem───3*[{imsettings-daem}]
        ├─irqbalance
        ├─ksmtuned───sleep
        ├─libvirtd───16*[{libvirtd}]
        ├─lsmd
        ├─lvmetad
        ├─master─┬─pickup
        │        └─qmgr
        ├─mission-control───3*[{mission-control}]
        ├─packagekitd───2*[{packagekitd}]
        ├─polkitd───6*[{polkitd}]
        ├─pulseaudio───2*[{pulseaudio}]
        ├─rngd
        ├─rpcbind
        ├─rsyslogd───2*[{rsyslogd}]
        ├─rtkit-daemon───2*[{rtkit-daemon}]
        ├─sd_dummy───{sd_dummy}
        ├─sd_espeak───4*[{sd_espeak}]
        ├─smartd
        ├─speech-dispatch───{speech-dispatch}
        ├─sshd─┬─sshd───bash───pstree
        │      └─sshd───bash───su───bash
        ├─systemd-journal
        ├─systemd-logind
        ├─systemd-udevd
        ├─tracker-store───7*[{tracker-store}]
        ├─tuned───4*[{tuned}]
        ├─udisksd───4*[{udisksd}]
        ├─upowerd───2*[{upowerd}]
        ├─vmhgfs-fuse───3*[{vmhgfs-fuse}]
        ├─vmtoolsd───{vmtoolsd}
        ├─vmtoolsd───3*[{vmtoolsd}]
        ├─wpa_supplicant
        └─xdg-permission-───2*[{xdg-permission-}]
```