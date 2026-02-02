# script to upload the currently built stm binaries to the board
# allocates gpios corresponding to the RST and BOOT0 and drives them to the proper levels
# docs for pinctrl are here: https://github.com/raspberrypi/utils/tree/master/pinctrl
# page 76 in this datasheet: https://www.st.com/content/ccc/resource/technical/document/application_note/b9/9b/16/3a/12/1e/40/0c/CD00167594.pdf/files/CD00167594.pdf/jcr:content/translations/en.CD00167594.pdf 
# requires previous installation of stm32cubeprogrammer, install via: https://www.st.com/en/development-tools/stm32cubeprog.html 
# reference: https://github.com/stm32duino/Arduino_Tools/blob/main/stm32CubeProg.sh 

#boot0, rst gpio defs in cougrc

#set boot0 high, reset, upload, set boot0 low, reset again
sudo pinctrl set $STM_BOOT0_GPIO op dh
sudo pinctrl set $STM_RST_GPIO op dl
sudo pinctrl set $STM_RST_GPIO op dh
#TODO: ensure UART configuration
#append stm32prog 
#UART 0 as of board version B, TODO: check if this connection works fs
PORT=/dev/ttyS0
STM32CP_CLI=STM32_Programmer.sh
export PATH="$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin":"$PATH"
FILEPATH="~/mcu_ws/stm/.pio/build/disco_f030r8/" #TODO: finish find the actual path
if ! command -v $STM32CP_CLI >/dev/null 2>&1; then
    echo "STM32_Programmer.sh not found, please install it and/or add it to PATH"
    exit 0
fi
#defaults for most options, quiet: no progress bar
${STM32CP_CLI} --connect port="${PORT}" "--erase" --quietMode --download "${FILEPATH}"
sudo pinctrl $STM_BOOT0_GPIO op dl
sudo pinctrl $STM_RST_GPIO op dl
sudo pinctrl $STM_RST_GPIO op dh