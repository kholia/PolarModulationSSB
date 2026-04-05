#!/usr/bin/env bash

sudo stty -F /dev/ttyACM0 1200

if test -f "$1"; then
  sudo picotool load -f -x $1
else
  echo "Cannot find file '$1'"
fi
