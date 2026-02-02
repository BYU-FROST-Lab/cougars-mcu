#!/bin/bash
# Created by Eli Gaskin Jan 2026
# detects whether hardware is on the CM5 (mainboard) or pi 5 (old) and builds for the respective solution
#using -f will allow you to force a certain build
TEENSY_DIRECTORY="~/mcu_ws/teensy"
STM_DIRECTORY="~/mcu_ws/stm"

if [[ "$1" = "-f" ]]; then
    read -p "forcing build, input 1 to build for the stm32 or 2 to build for the teensy " buildop
    case $buildop in
    1)
        echo "building for the stm32"
        cd $STM_DIRECTORY
        ;;
    2)
        cd $TEENSY_DIRECTORY
        echo "building for the teensy"
        ;;
    *)
        echo "bad input $buildop"
        exit 0
    esac
else
    if [[ $UCONTROLLER = "STM" ]];
        echo "building for stm32"
        cd $STM_DIRECTORY
    else
        echo "building for teensy 4.1"
        cd $TEENSY_DIRECTORY
    fi
fi
pio run #build