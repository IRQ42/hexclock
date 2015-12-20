CC= avr-gcc
OBJCOPY= avr-objcopy
AVRDUDE= avrdude

PART=attiny2313
MCU=attiny2313a


PROGRAMMER= usbtiny
PROGRAMMER_PORT = usb
SEGMENTS= -j .text -j .data
HEX= ihex
STD= -std=c99
WARN= -Wall -Werror
OPT= -Os
LDFLAGS=-Wl,-Map,hexclock.map
CFLAGS=-$(STD) $(WARN) $(OPT) -mmcu=$(MCU) -fshort-enums
AVRDUDE_FLAGS= -v -p $(PART) -c $(PROGRAMMER) -P $(PROGRAMMER_PORT)
AVRDUDE_MEM= -U flash:w:$(HEX):i

SRCS= hexclock.c
OBJS= $(SRCS:.c=.o)
ELF= hexclock.elf
HEX= $(ELF:.elf=.hex)
#removed .bss

all: $(HEX)

$(HEX): $(ELF)
	$(OBJCOPY) $(SEGMENTS) -O ihex $(ELF) $(HEX)

$(ELF): $(OBJS)
	$(CC) $(CFLAGS) -o $(ELF) $(OBJS) $(LDFLAGS)

.c.o:   $(SRCS) 
	$(CC) $(CFLAGS) -c  $< -o $@

.PHONY: all clean program asm

clean:
	$(RM) $(OBJS) $(ELF) $(HEX) *.map *.s

program:
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_MEM)

asm:
	$(CC) $(CFLAGS) -S -o hexclock.s $(SRCS)
