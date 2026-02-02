#!/bin/bash
# Created by Eli Gaskin Jan 2026
# detects whether hardware is on the CM5 (mainboard) or pi 5 (old) and builds for the respective solution
#using -f will allow you to force a certain upload method

# TODO: IMPLEMENT STM UPLOADING, teensy testing

TEENSY_DIRECTORY="~/mcu_ws/teensy"
STM_DIRECTORY="~/mcu_ws/stm"
if [ "$1" = "-f" ]; then
    read -p "forcing upload, input 1 to build for the stm32 or 2 to build for the teensy " buildop
    case $buildop in
    1)
        echo "uploading to the stm32"
        cd $STM_DIRECTORY
        ;;
    2)
        cd $TEENSY_DIRECTORY
        echo "uploading to the teensy whatever's in the PIO build directory"
        python3 ~/gpio/gpio_tools/program.py
        tycmd upload $TEENSY_DIRECTORY/.pio/build/teensy41/firmware.hex
        ;;
    *)
        echo "bad input $buildop"
        exit 0
    esac
else
    if [[-v UCONTROLLER ]] then
        if [[ $UCONTROLLER = "STM" ]];
            echo "building for stm32"
            cd $STM_DIRECTORY
        else
            echo "building for teensy 4.1"
            cd $TEENSY_DIRECTORY
            case $1 in
                "")
                    python3 ~/gpio/gpio_tools/program.py
                    tycmd upload $TEENSY_DIRECTORY/.pio/build/teensy41/firmware.hex
                    ;;
                *)
                    python3 ~/gpio/gpio_tools/program.py
                    cd $TEENSY_DIRECTORY/../firmware_options
                    tycmd upload $1
                    ;;
            esac
        fi
    else
        echo "UCONTROLLER not set, defaulting to building for teensy 4.1"
        case $1 in
            "")
                python3 ~/gpio/gpio_tools/program.py
                tycmd upload $TEENSY_DIRECTORY/.pio/build/teensy41/firmware.hex
                ;;
            *)
                python3 ~/gpio/gpio_tools/program.py
                cd $TEENSY_DIRECTORY/../firmware_options
                tycmd upload $1
                ;;
        esac
    fi

fi
