#!/bin/bash
# Created by Eli Gaskin Jan 2026
#uploads firmware to the target microcontroller set by $UCONTROLLER in cougrc, forcable using -f


# TODO: testing

TEENSY_DIRECTORY=/home/frostlab/cougars-frost/mcu_ws/teensy
STM_DIRECTORY=/home/frostlab/cougars-frost/mcu_ws/stm

if command -v pio >/dev/null 2>&1; then
    PIO_CMD="pio"
elif [[ -x "$HOME/.platformio/penv/bin/platformio" ]]; then
    PIO_CMD="$HOME/.platformio/penv/bin/platformio"
else
    echo "PlatformIO not found. Install it or add pio/platformio to PATH."
    exit 1
fi
if [[ "$1" = "-f" ]]; then
    read -p "forcing upload, input 1 to upload to the stm32 or 2 for the teensy " buildop
    case $buildop in
    1)
        echo "uploading to the stm32"
        cd $STM_DIRECTORY
        # build arg to signal that it's being built on the actual board/docker container
        "$PIO_CMD" run -t upload --program-arg "ONBOARD" 
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
    if [[ $UCONTROLLER = "STM" ]]; then
        echo "building for stm32"
        cd $STM_DIRECTORY
        # build arg to signal that it's being built on the actual board/docker container
        case $1 in
            "")
                "$PIO_CMD" run -t upload --program-arg "ONBOARD" 
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
