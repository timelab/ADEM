ARDUINO_PATH = $(CURDIR)/Arduino
ARDUINO15_PATH = $(HOME)/.arduino15
TARGET_DIR = $(CURDIR)/Build
SERIAL_PORT = /dev/ttyUSB0
SERIAL_BAUD = 921600
ESP8266_BAUD = 38400
#ESP8266_BAUD = 74880
#ESP8266_BAUD = 115200
SKETCH = adem.ino

BOARD = esp8266:esp8266:thing
HWTYPE= esp8266

ESPTOOL = $(ARDUINO15_PATH)/packages/$(HWTYPE)/tools/esptool/0.4.8/esptool
CTAGS = $(ARDUINO_PATH)/tools-builder/ctags/5.8-arduino10

PREFS = -prefs=tools.ctags.path=$(CTAGS)

all:
	@if [ ! -d "$(ARDUINO_PATH)" ]; then echo "Please make a symlink from your Arduino installation to $(ARDUINO_PATH)."; false; fi
	mkdir -p $(TARGET_DIR)
	$(ARDUINO_PATH)/arduino-builder -compile \
		-fqbn $(BOARD) -verbose -debug-level 9 ${PREFS} \
		-hardware $(ARDUINO_PATH)/hardware \
		-hardware $(ARDUINO15_PATH)/packages \
		-tools $(ARDUINO_PATH)/tools \
		-tools $(ARDUINO15_PATH)/packages \
		-libraries ./libraries \
		-build-path $(TARGET_DIR) \
		adem/adem.ino

flash:
	$(ESPTOOL) -v -cd ck -cb $(SERIAL_BAUD) -cp $(SERIAL_PORT) -ca 0x00000 -cf $(TARGET_DIR)/$(SKETCH).bin

upload: all flash

clean:
	rm -rf $(TARGET_DIR)

monitor:
#	stty -F $(SERIAL_PORT) $(ESP8266_BAUD)
#	screen $(SERIAL_PORT) $$(stty speed <$(SERIAL_PORT))
	screen $(SERIAL_PORT) $(ESP8266_BAUD)