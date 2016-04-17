ARDUINO_PATH = $(CURDIR)/Arduino
ARDUINO15_PATH = $(HOME)/.arduino15
TARGET_DIR = $(CURDIR)/Build

SERIAL_PORT := /dev/ttyUSB0
#SERIAL_BAUD := 38400
SERIAL_BAUD := 74880
#SERIAL_BAUD := 115200
FLASH_BAUD := 921600

SKETCH = adem/adem.ino

BOARD = esp8266:esp8266:thing
HWTYPE = esp8266

ESPTOOL = $(ARDUINO15_PATH)/packages/$(HWTYPE)/tools/esptool/0.4.8/esptool
CTAGS = $(ARDUINO_PATH)/tools-builder/ctags/5.8-arduino10

CXX_FLAGS := -DDEBUG
PREFS = -prefs=build.debug_level="$(CXX_FLAGS)" -prefs=tools.ctags.path="$(CTAGS)"

SKETCHES = $(find $(CURDIR) -name *.ino)

.PHONY: all flash upload tests

all:
	@if [ ! -d "$(ARDUINO_PATH)" ]; then echo "Please make a symlink from your Arduino installation to $(ARDUINO_PATH)."; false; fi
	mkdir -p $(TARGET_DIR)
	$(ARDUINO_PATH)/arduino-builder -compile \
		-fqbn $(BOARD) \
		-verbose -debug-level 9 $(PREFS) \
		-hardware $(ARDUINO_PATH)/hardware \
		-hardware $(ARDUINO15_PATH)/packages \
		-tools $(ARDUINO_PATH)/tools \
		-tools $(ARDUINO15_PATH)/packages \
		-libraries ./libraries \
		-build-path $(TARGET_DIR) \
		$(SKETCH)

flash:
	tput reset > $(SERIAL_PORT)
	$(ESPTOOL) -v -cd nodemcu -cb $(FLASH_BAUD) -cp $(SERIAL_PORT) -ca 0x00000 -cf $(TARGET_DIR)/$(shell basename $(SKETCH)).bin
	tput reset > $(SERIAL_PORT)

upload: all flash

tests:
	$(foreach SKETCH,$(SKETCHES), make SKETCH=$(SKETCH); )

clean:
	rm -rf $(TARGET_DIR)

serial:
#	stty -F $(SERIAL_PORT) speed $(SERIAL_BAUD) cs8 -cstopb -parenb
#	stty -F $(SERIAL_PORT) speed $(SERIAL_BAUD)
	@echo "Serial speed is $(SERIAL_BAUD)"
	cat <$(SERIAL_PORT)

monitor:
#	stty -F $(SERIAL_PORT) speed $(SERIAL_BAUD) cs8 -cstopb -parenb
#	screen $(SERIAL_PORT) $$(stty speed <$(SERIAL_PORT))
	@echo "Serial speed is $(SERIAL_BAUD)"
	screen $(SERIAL_PORT) $(SERIAL_BAUD)