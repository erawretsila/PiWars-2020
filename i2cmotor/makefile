#makefile for ATTiny4313


DEVICE  = attiny2313a
DEV=attiny2313
CLOCK=8000000
PROGRAMMER = -c usbasp
FILENAME = motor
OBJECTS = $(FILENAME) usi_i2c_slave

#for ATTiny 85
#see http://engbedded.com/fusecalc/

FUSES = -U lfuse:w:0xe4:m -U hfuse:w:0x9f:m -Uefuse:w:0xff:m

AVRDUDE = avrdude $(PROGRAMMER) -p $(DEV)
COMPILE = avr-gcc -Wall -Os -DF_CPU=$(CLOCK) -mmcu=$(DEVICE) --short-enums

#Symbolivc targets

all:	$(FILENAME).hex $(FILENAME).eep

.c.o:
	$(COMPILE) -c $< -o $@

.S.o:
	$(COMPILE) -x assembler-with-cpp -c $< -o $@

.c.s:
	$(COMPILE) -S $< -o $@

flash: all
	$(AVRDUDE) -U flash:w:$(FILENAME).hex:i
	
fuse: all
	$(AVRDUDE) -B 1000 $(FUSES)

phoney:
eeprom:
	$(AVRDUDE) -U eeprom:w:$(FILENAME).eep:i
	
install: fuse flash eeprom


clean:
	rm -f *.hex *.elf *.o

$(FILENAME).elf: $(FILENAME).o usi_i2c_slave.o 
	$(COMPILE) -o $(FILENAME).elf motor.o usi_i2c_slave.o
	avr-objdump -d -S $@ >$(FILENAME).lss

$(FILENAME).hex: $(FILENAME).elf
	rm -f $(FILENAME).hex
	avr-objcopy -j .text -j .data -O ihex $(FILENAME).elf $(FILENAME).hex
	avr-size --format=avr --mcu=$(DEVICE) $(FILENAME).elf

$(FILENAME).eep: $(FILENAME).elf
	rm -f $(FILENAME).eep
	-avr-objcopy -j .eeprom --change-section-lma .eeprom=0 --no-change-warnings -O ihex $< $@
#	avr-objcopy -j .eeprom -O ihex $(FILENAME).elf $(FILENAME).eep
	
