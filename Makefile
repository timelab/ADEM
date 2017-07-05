ARDUINO_PATH = $(CURDIR)/Arduino
ARDUINO15_PATH = $(CURDIR)/arduino15
BUILD_PATH = $(CURDIR)/Build

SKETCH = adem/adem/adem.ino
SKETCH_NAME = $(shell basename $(SKETCH))
SKETCH_DIR = $(shell dirname $(SKETCH))

SERIAL_BAUD := 38400

### Build using debug output by default
CFLAGS := -DDEBUG

### Workaround for bug https://github.com/esp8266/Arduino/pull/2279
CFLAGS += -DDEBUG_OUTPUT=Serial

### Set hardware type based on SKETCH
ifeq ($(SKETCH_DIR),i2c-slave)
HWTYPE := uno-pro-mini
else ifeq ($(SKETCH_DIR),people/dagwieers/arduino_pro_mini_test)
HWTYPE := uno-pro-mini
else ifeq ($(SKETCH_DIR),people/kavers1/i2c-gps-nav)
HWTYPE := uno-pro-mini
else
HWTYPE := sparkfun-esp8266-thing
endif

### Sparkfun ESP8266 Thing
ifeq ($(HWTYPE),sparkfun-esp8266-thing)
BOARD = esp8266:esp8266:thing
SERIAL_PORT := /dev/ttyUSB0
#SERIAL_BAUD := 74880
#FLASH_BAUD := 921600
FLASH_BAUD := 460800
UPLOAD_CMD = "$(ARDUINO15_PATH)/packages/esp8266/tools/esptool/0.4.9/esptool" -v -cd nodemcu -cb $(FLASH_BAUD) -cp $(SERIAL_PORT) -ca 0x00000 -cf "$(BUILD_PATH)/$(SKETCH_NAME).bin"
endif

### Arduino UNO Pro Mini
ifeq ($(HWTYPE),uno-pro-mini)
BOARD = arduino:avr:pro:cpu=8MHzatmega328
SERIAL_PORT := /dev/ttyUSB1
SERIAL_BAUD := 115200
FLASH_BAUD := 57600
UPLOAD_CMD = "$(ARDUINO_PATH)/hardware/tools/avr/bin/avrdude" -C $(ARDUINO_PATH)/hardware/tools/avr/etc/avrdude.conf -v -p atmega328p -c arduino -P $(SERIAL_PORT) -b $(FLASH_BAUD) -D -U flash:w:"$(BUILD_PATH)/$(SKETCH_NAME).hex"
endif

### Arduino UNO ATMEGA328
ifeq ($(HWTYPE),uno)
#BOARD = arduino:avr:diecimila:cpu=atmega328
BOARD = arduino:avr:uno:cpu=atmega328
SERIAL_PORT := /dev/ttyACM3
FLASH_BAUD := 115200
UPLOAD_CMD = "$(ARDUINO_PATH)/hardware/tools/avr/bin/avrdude" -C $(ARDUINO_PATH)/hardware/tools/avr/etc/avrdude.conf -v -p atmega328p -c arduino -P $(SERIAL_PORT) -b $(FLASH_BAUD) -D -U flash:w:"$(BUILD_PATH)/$(SKETCH_NAME).hex"
endif

CTAGS = $(ARDUINO_PATH)/tools-builder/ctags/5.8-arduino11
PREFS = --prefs=build.debug_level="$(CFLAGS)" --prefs=tools.ctags.path="$(CTAGS)"

### Add project custom libraries/ directory
ifneq ($(wildcard $(SKETCH_DIR)/libraries/.*),)
    LIBRARIES = $(SKETCH_DIR)/libraries/
else
    LIBRARIES = adem/libraries/
endif

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
		--libraries="$(LIBRARIES)" \
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
#	stty <$(SERIAL_PORT) $(SERIAL_BAUD)
	cat <$(SERIAL_PORT)

monitor:
	@echo "Serial speed is $(SERIAL_BAUD)"
	-setserial -v $(SERIAL_PORT) spd_cust divisor $$(( 24000000 / ( 2 * $(SERIAL_BAUD) ) ))
#	stty -F $(SERIAL_PORT) ispeed $(SERIAL_BAUD) ospeed $(SERIAL_BAUD) cs8 -cstopb parenb
#	stty <$(SERIAL_PORT) $(SERIAL_BAUD)
	screen -fn $(SERIAL_PORT) $(SERIAL_BAUD),cs8,ixon,ixoff,-istrip,-ctsrts,-dsrdtr
