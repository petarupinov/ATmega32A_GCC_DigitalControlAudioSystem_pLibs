rm -r default/*
cp Makefile default/
# FLASH
#sudo avrdude -p m32 -c stk500v2 -P /dev/ttyUSB0 -F -U flash:w:default/ATmega32A_GCC_DigitalControlAudioSystem_pLibs.hex
#sudo avrdude -p m32 -c stk500v2 -P /dev/ttyUSB0 -F -U flash:w:ATmega32A_GCC_DigitalControlAudioSystem_pLibs.hex

# UART TERMIANL:
# ls /dev/tty.*
# sudo apt-get install screen
# sudo screen /dev/ttyUSB1 9600


