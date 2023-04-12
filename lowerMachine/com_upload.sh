#! /usr/bin/bash
arduino-cli compile --verbose --fqbn arduino:avr:mega lowerMachine.ino
arduino-cli upload -p /dev/ttyACM0 --verbose --fqbn arduino:avr:mega lowerMachine.ino