#
# embedXcode
# ----------------------------------
# Embedded Computing on Xcode
#
# Copyright © Rei VILO, 2010-2015
# All rights reserved
# http://embedxcode.weebly.com
#
# Last update: Jul 31, 2015 release 300


# Libraries
# ----------------------------------
# Declare application Arduino/chipKIT/Digispark/Energia/Maple/Microduino/Teensy/Wiring
# and user's libraries used 
# Short-listing libraries speeds-up building
# Typing = 0 takes none (default for application and user)
# Otherwise, leaving empty considers all (default for local)

# List application Arduino/chipKIT/Digispark/Energia/Maple/Microduino/Teensy/Wiring libraries here
# default = 0 = none
#
# ARDUINO = Bridge Esplora Ethernet Firmata GSM 
#	LiquidCrystal Robot_Control Robot_Motor RobotIRremote SD Servo 
#	SpacebrewYun Stepper Temboo TFT WiFi 
#
APP_LIBS_LIST = SPI

# List user's libraries here 
# default = 0 = none
#
#
USER_LIBS_LIST = Adafruit_nRF8001 MPU6050 i2c_t3 I2Cdev

# List core, application and user's libraries to be excluded
# For example, WiFi may crash on Arduino 1.0.2, Esplora on Arduino 1.0.3, Firmata on Teensy 3.0,
#   OneWire on MPIDE 0023, HardwareSerial may conflict with MarlinSerial
#
#EXCLUDE_LIBS = Firmata WiFi Esplora OneWire Robot_Control Robot_Control/utility Robot_Motor


# Parameters
# ----------------------------------
# Xcode takes BOARD_TAG and BOARD_PORT from the .xcconfig file
# For other IDEs than Xcode,

# BOARD_TAG is required
# If not defined, error BOARD_TAG not defined
#
#BOARD_TAG  = mega2560

# BOARD_PORT is optional
# If not defined, BOARD_PORT = /dev/tty.usb* (default)
#
#BOARD_PORT = /dev/tty.usbmodem*

# Xcode takes SKETCHBOOK_DIR from preferences.txt
# If not defined, each IDE has its own SKETCHBOOK_DIR (default)
# To share the same SKETCHBOOK_DIR along all IDEs, define
#
#SKETCHBOOK_DIR =

# SERIAL_BAUDRATE for the serial console, 9600 by default
# Uncomment and specify another speed
#
#SERIAL_BAUDRATE = 115200


# Miscellaneous
# ----------------------------------
# Manage path with space in the name
#
CURRENT_DIR   := $(shell pwd)
CURRENT_DIR   := $(shell echo '$(CURRENT_DIR)' | sed 's/ /\\\ /g')

# C-compliant project name
#
PROJECT_NAME_AS_IDENTIFIER = Smart_Arm_Band
SKETCH_EXTENSION = ino

MAKEFILE_PATH  = $(CURRENT_DIR)/Makefiles
UTILITIES_PATH = $(CURRENT_DIR)/Utilities

include $(MAKEFILE_PATH)/Step1.mk


