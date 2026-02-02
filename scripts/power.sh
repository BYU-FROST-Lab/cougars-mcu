#!/bin/bash
# Created by Eli Gaskin Jan 2026
# specifying "on" will reset the microcontroller, specifying "off" will simply power off/reset the mcu
# - Specify a power state using 'bash power.sh <state>'

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

case $1 in
    "on")
        if [[ $UCONTROLLER = "STM" ]];
            echo "resetting stm32, turning on via pin $STM_RST_GPIO"
            sudo pinctrl set $STM_RST_GPIO op dl
            sleep 0.1
            sudo pinctrl set $STM_RST_GPIO op dh
        else
            echo "resetting, turning on Teensy via pin $POWER_PIN"
            sudo pinctrl set $POWER_PIN op dl
            sleep 0.1
            sudo pinctrl set $POWER_PIN op dh
        fi

        ;;
    "off")
        if [[ $UCONTROLLER = "STM" ]];
            echo "turning off stm32 via pin $STM_RST_GPIO"
            sudo pinctrl set $STM_RST_GPIO op dl
        else
            echo "turning off Teensy via pin $POWER_PIN"
            sudo pinctrl set $POWER_PIN op dl
        fi
        ;;
    *)
        printError "No power state specified"
        printError "Specify a power state using 'bash power.sh <state on/off>'"
        ;;
esac
