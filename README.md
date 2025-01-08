# bens_binary_clock

Code for an ATMega48 binary clock
by Ben Kaden
working condition 08.01.25
includes sleepmode, turned off peripherals and scaleable PWM
____________________________________________________________

Tutorial for avrdude on windows:

https://tinusaur.com/guides/avr-gcc-toolchain/

you can find guides for other OS there too.
____________________________________________________________

shortcut code for avrdude:

1:avr-gcc -mmcu=atmega48 -DF_CPU=1000000UL -Wcpp -Os -o main.elf main.c
2:avr-objcopy -O ihex -R .eeprom main.elf main.hex
3:avrdude -c usbasp -p m48 -U flash:w:main.hex:i
____________________________________________________________
