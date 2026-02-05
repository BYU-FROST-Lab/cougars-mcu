echo "Entering Teensy board program mode . . ."
sudo pinctrl set $PROGRAM_PIN op dl
sleep 0.1
sudo pinctrl set $PROGRAM_PIN op dh
sleep 1
echo "[COMPLETE] Entered Teensy board program mode"