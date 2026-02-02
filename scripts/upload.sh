#!/bin/bash
# Created by Eli Gaskin Jan 2026
# chooses whether to upload for the STM32 or teensy based off of UCONTROLLER variable set in cougrc
#using -f will allow you to force a certain upload method


# TODO: testing

TEENSY_DIRECTORY="~/mcu_ws/teensy"
STM_DIRECTORY="~/mcu_ws/stm"
if [[ "$1" = "-f" ]]; then
    read -p "forcing upload, input 1 to build for the stm32 or 2 to build for the teensy " buildop
    case $buildop in
    1)
        echo "uploading to the stm32"
        cd $STM_DIRECTORY
        # build arg to signal that it's being built on the actual board/docker container
        pio run -t upload --program-arg "ONBOARD" 
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
    if [[ $UCONTROLLER = "STM" ]];
        echo "building for stm32"
        cd $STM_DIRECTORY
        # build arg to signal that it's being built on the actual board/docker container
        case $1 in
            "")
                pio run -t upload --program-arg "ONBOARD" 
                ;;
            *)
                bash $STM_DIRECTORY/upload.sh $1
                ;;
        esac
    else
        echo "building for teensy 4.1"
        cd $TEENSY_DIRECTORY
        case $1 in
            "")
                bash $TEENSY_DIRECTORY/program_mode.sh
                tycmd upload $TEENSY_DIRECTORY/.pio/build/teensy41/firmware.hex
                ;;
            *)
                bash $TEENSY_DIRECTORY/program_mode.sh
                cd $TEENSY_DIRECTORY/../firmware_options/teensy
                tycmd upload $1
                ;;
        esac
    fi

fi
