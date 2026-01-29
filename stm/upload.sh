# script to upload the currently built stm binaries to the board
# allocates gpios corresponding to the RST and BOOT0 and drives them to the proper levels
# docs for pinctrl are here: https://github.com/raspberrypi/utils/tree/master/pinctrl
# page 76 in this datasheet: https://www.st.com/content/ccc/resource/technical/document/application_note/b9/9b/16/3a/12/1e/40/0c/CD00167594.pdf/files/CD00167594.pdf/jcr:content/translations/en.CD00167594.pdf 
#requires previous installation of stm32cubeprogrammer
# reference: https://github.com/stm32duino/Arduino_Tools/blob/main/stm32CubeProg.sh 

sudo pinctrl 4,6 op dl
#TODO: ensure UART configuration
#append stm32prog 
#UART 0 as of board version B, TODO: check if this connection works fs
PORT=/dev/ttyS0
STM32CP_CLI=STM32_Programmer.sh
export PATH="$HOME/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin":"$PATH"
# build arg to signal that it's being built on the actual board/docker container
pio run --program-arg "ONBOARD" 
#defaults for most options, quiet: no progress bar
${STM32CP_CLI} --connect port="${PORT}" "--erase" --quietMode --download "${FILEPATH}"