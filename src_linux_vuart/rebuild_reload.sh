#!/bin/bash
set -e
sudo rmmod vuart 2>/dev/null || true
make clean >/dev/null 2>&1 || true
make
sudo insmod vuart.ko
sleep 0.5
sudo chmod 666 /dev/vuart0 /dev/vuart1
ls -la /dev/vuart0 /dev/vuart1
lsmod | grep vuart
dmesg | tail -5
