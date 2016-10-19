ARDUINO_PATH = $(CURDIR)/Arduino
ARDUINO15_PATH = $(CURDIR)/arduino15
BUILD_PATH = $(CURDIR)/Build

SKETCH = adem/adem.ino
SKETCH_DIR = $(shell dirname $(SKETCH))

SERIAL_BAUD := 38400
CFLAGS := -DDEBUG

### Set hardware type based on SKETCH
ifeq ($(SKETCH_DIR),i2c-slave)
HWTYPE := atmega328
else ifeq ($(SKETCH_DIR),people/kavers1/i2c-gps-nav)
HWTYPE := atmega328
else
HWTYPE := esp8266
endif

### Sparkfun ESP8266 Thing
ifeq ($(HWTYPE),esp8266)
BOARD = esp8266:esp8266:thing
HWTYPE = esp8266
SERIAL_PORT := /dev/ttyUSB0
FLASH_BAUD := 921600
BUILD_IMAGE = "$(BUILD_PATH)/$(shell basename $(SKETCH)).bin"
UPLOAD_CMD = "$(ARDUINO15_PATH)/packages/$(HWTYPE)/tools/esptool/0.4.9/esptool" -v -cd nodemcu -cb $(FLASH_BAUD) -cp $(SERIAL_PORT) -ca 0x00000 -cf "$(BUILD_IMAGE)"
endif

### Arduino UNO ATMEGA328
ifeq ($(HWTYPE),atmega328)
#BOARD = arduino:avr:diecimila:cpu=atmega328
BOARD = arduino:avr:uno:cpu=atmega328
SERIAL_PORT := /dev/ttyACM3
HWTYPE = atmega328
FLASH_BAUD := 115200
BUILD_IMAGE = "$(BUILD_PATH)/$(shell basename $(SKETCH)).hex"
UPLOAD_CMD = "$(ARDUINO_PATH)/hardware/tools/avr/bin/avrdude" -C $(ARDUINO_PATH)/hardware/tools/avr/etc/avrdude.conf -v -p atmega328p -c arduino -P $(SERIAL_PORT) -b $(FLASH_BAUD) -D -U flash:w:"$(BUILD_IMAGE)"
endif

CTAGS = $(ARDUINO_PATH)/tools-builder/ctags/5.8-arduino10

# Define DEBUG_OUTPUT for internal ESP DNS library when DEBUG is set
CFLAGS += -DDEBUG_OUTPUT=Serial
PREFS = --prefs=build.debug_level="$(CFLAGS)" --prefs=tools.ctags.path="$(CTAGS)"

SKETCHES = $(find $(CURDIR) -name *.ino)

.PHONY: all flash upload tests

all: build

build:
	@if [ ! -d "$(ARDUINO_PATH)" ]; then echo "Please make a symlink from your Arduino installation to $(ARDUINO_PATH)."; false; fi
	mkdir -p "$(BUILD_PATH)"
	"$(ARDUINO_PATH)/arduino-builder" --compile \
		--fqbn=$(BOARD) \
		--verbose --debug-level=9 $(PREFS) \
		--hardware="$(ARDUINO_PATH)/hardware" \
		--hardware="$(ARDUINO15_PATH)/packages" \
		--tools="$(ARDUINO_PATH)/tools" \
		--tools="$(ARDUINO15_PATH)/packages" \
		--libraries=$(SKETCH_DIR)/libraries \
		--build-path="$(BUILD_PATH)" \
		$(SKETCH)

flash:
	$(UPLOAD_CMD)

upload: build flash

tests:
	$(foreach SKETCH,$(SKETCHES), make SKETCH=$(SKETCH); )

clean:
	rm -rf $(BUILD_PATH)

serial:
	@echo "Serial speed is $(SERIAL_BAUD)"
	-setserial -v $(SERIAL_PORT) spd_cust divisor $$(( 24000000 / ( 2 * $(SERIAL_BAUD) ) ))
#	stty -F $(SERIAL_PORT) ispeed $(SERIAL_BAUD) ospeed $(SERIAL_BAUD) cs8 -cstopb parenb
#	stty -F $(SERIAL_PORT) ispeed $(SERIAL_BAUD) ospeed $(SERIAL_BAUD)
#	stty <$(SERIAL_PORT)
	cat <$(SERIAL_PORT)
#	cu --line $(SERIAL_PORT) --speed $(SERIAL_BAUD)

monitor:
	@echo "Serial speed is $(SERIAL_BAUD)"
	-setserial -v $(SERIAL_PORT) spd_cust divisor $$(( 24000000 / ( 2 * $(SERIAL_BAUD) ) ))
#	stty -F $(SERIAL_PORT) ispeed $(SERIAL_BAUD) ospeed $(SERIAL_BAUD) cs8 -cstopb parenb
#	stty -F $(SERIAL_PORT) ispeed $(SERIAL_BAUD) ospeed $(SERIAL_BAUD)
	stty <$(SERIAL_PORT)
	screen -fn $(SERIAL_PORT) $(SERIAL_BAUD),cs8,ixon,ixoff,-istrip,-ctsrts,-dsrdtr
