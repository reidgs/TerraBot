ARDUINO_DIR = /usr/share/arduino
ARDUINO_PORT = /dev/ttyACM0
ARDUINO_SKETCHBOOK = ${HOME}/Sketchbook
AVRDUDE_CONF = ${ARDUINO_DIR}/hardware/tools/avrdude.conf
AVRDUDE_ARD_PROGRAMMER = wiring
AVRDUDE_ARD_BAUDRATE = 115200

#ARDUINO_LIBS =TCA9548 DHT20 HX711 Wire dhtnew
#ARDUINO_LIBS = dhtnew HX711
ARDUINO_LIBS = dhtnew HX711 Wire A02YYUW

BOARD_TAG = mega
MCU = atmega2560
CPPFLAGS = -DUSE_A02YYUW -DUSE_BOTH
include /usr/share/arduino/Arduino.mk
