#!/bin/bash
#   ensure_loaded.sh PLUG_DEV PEER_DEV SUDO_PASSWORD
#
# Rebuilds vuart.ko when missing, older than vuart.c or built for another
# kernel. Reloads when the loaded srcversion differs, killing a stale
# vuart_loopback.py first. Loads it if absent. Then chmods both nodes 666
# and reports whether the ttys have a bound parent device.
#
# sudo -S with the host login password, piped in since there is no tty.
set -euo pipefail
plug=$1 peer=$2 pw=$3
cd "$(dirname "$0")"
sudo_() { printf '%s\n' "$pw" | sudo -S -p '' "$@"; }

kver=$(uname -r)
need_build=0
if [ ! -f vuart.ko ]; then
    echo "vuart.ko missing"; need_build=1
elif [ vuart.c -nt vuart.ko ]; then
    echo "vuart.c is newer than vuart.ko"; need_build=1
elif [ "$(modinfo -F vermagic vuart.ko | cut -d' ' -f1)" != "$kver" ]; then
    echo "vuart.ko was built for $(modinfo -F vermagic vuart.ko | cut -d' ' -f1), running $kver"; need_build=1
fi
if [ "$need_build" = 1 ]; then
    make clean >/dev/null 2>&1 || true
    make
fi

want=$(modinfo -F srcversion vuart.ko)
if [ -d /sys/module/vuart ]; then
    have=$(cat /sys/module/vuart/srcversion 2>/dev/null || echo unknown)
    if [ "$have" != "$want" ]; then
        echo "loaded vuart srcversion $have != vuart.ko $want -- reloading"
        pkill -f "[v]uart_loopback.py" 2>/dev/null || true
        sleep 1
        if ! sudo_ rmmod vuart; then
            echo "rmmod vuart failed -- something still holds a vuart node open:"
            fuser -v "$plug" "$peer" 2>&1 || true
            exit 1
        fi
        sudo_ insmod vuart.ko
        sleep 0.5
    else
        echo "vuart loaded, srcversion $have matches vuart.ko"
    fi
else
    echo "vuart not loaded -- loading"
    sudo_ insmod vuart.ko
    sleep 0.5
fi

sudo_ chmod 666 "$plug" "$peer" 2>/dev/null || true
if [ ! -c "$plug" ] || [ ! -c "$peer" ]; then
    echo "MISSING: $plug and/or $peer still absent after insmod"
    lsmod | grep vuart || echo "vuart not in lsmod"
    dmesg | tail -20
    exit 1
fi
ls -l "$plug" "$peer"

for d in "$plug" "$peer"; do
    n=$(basename "$d")
    drv=$(readlink "/sys/class/tty/$n/device/driver" 2>/dev/null || true)
    if [ -n "$drv" ]; then
        echo "$n: parent driver $(basename "$drv") -- listed by 86Box's host-serial picker"
    else
        echo "$n: no parent driver -- 86Box's host-serial picker shows None (path in 86box.cfg still works)"
    fi
done
