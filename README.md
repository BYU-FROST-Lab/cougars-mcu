Formerly cougars-teensy
Code for microcontrollers meant to control several basic tasks in the cougars project such as handling servos, monitoring battery voltage and leaks all via NMEA messages.
Switching between the teensy and stm build modes relies on the bash variable UCONTROLLER that should be in the cougrc of the vehicle
Uses PlatformIO to build and upload code in most cases
firmware_options should contain several working builds of the firmware
Usage:
scripts/build.sh: builds firmware for the target
scripts/upload.sh: uploads firmware to the target
scripts/power.sh: resets the target or turns the target off

written by Braden Meyers, Nelson Durrant, and Eli Gaskin