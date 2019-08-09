#makefile for ATTiny4313

DEVICE  = attiny2313
CLOCK=8000000
PROGRAMMER = -c usbasp
FILENAME = motor
OBJECTS = $(FILENAME)

#for ATTiny 85
#see http://engbedded.com/fusecalc/

FUSES = -U lfuaae:w:0xe4:m -U hfuse:w:0x9f:m -Uefuse:w:0xff:m

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEVICE) 
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE)

#Symbolivc targets

all:	$(FILENAME).hex

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

.c.s:
	$(COMPILE) -S $< -o $@

flash: all
	$(AVRDUDE) -U flash:w:$(FILENAME).hex:i

fuse:
	$(AVRDUDE) $(FUSES)

install: flash fuse

clean:
	rm -f *.hex *.elf *.o

$(FILENAME).elf: $(FILENAME).o usi_i2c_slave.o 
	$(COMPILE) -o $(FILENAME).elf motor.o usi_i2c_slave.o
	avr-objdump -d -S $@ >$(FILENAME).lss

$(FILENAME).hex: $(FILENAME).elf
	rm -f $(FILENAME).hex
	avr-objcopy -j .text -j .data -O ihex $(FILENAME).elf $(FILENAME).hex
	avr-size --format=avr --mcu=$(DEVICE) $(FILENAME).elf

