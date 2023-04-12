#! /usr/bin/bash
arduino-cli compile --fqbn arduino:avr:mega lowerMachine
arduino-cli upload -p /dev/ttyACM0 --verbose --fqbn arduino:avr:mega lowerMachine