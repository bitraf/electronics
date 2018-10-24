
TARGET=attiny2313
NAME=main
CFLAGS += -fdata-sections -ffunction-sections -Wall -fshort-enums -mcall-prologues -Os -std=c99
LINKFLAGS += -Wl,-gc-sections -Wl,--relax -flto

all: compile link hex size

link:
	avr-gcc -mmcu=attiny2313 ${LINKFLAGS} -o $(NAME).elf main.o

compile:
	avr-gcc  ${CFLAGS} -mmcu=attiny2313 -c main.c

dump:
	avr-objdump -h -S main.elf > main.lst

map:
	 avr-gcc -g -mmcu=$(TARGET) -Wl,-Map,${NAME}.map -o ${NAME}.elf ${NAME}.o

hex:
	 avr-objcopy -j .text -j .data -O ihex ${NAME}.elf ${NAME}.hex

eeprom:
	avr-objcopy -j .eeprom --change-section-lma .eeprom=0 -O ihex ${NAME}.elf ${NAME}.eep

size:
	avr-size ${NAME}.elf

flash:
	 avrdude -c stk500  -p t2313 -P /dev/ttyUSB0 -U flash:w:main.hex

font:
	python raw.py
clean:
	rm -rf *.o ${NAME}.elf ${NAME}.hex

asm:
	
