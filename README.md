# OTTOpov Project
# 2017-11-09
#########################################################################
# H/W                                                                   #
#          - Arduino Nano                                               #
#          - Hall Effect Sensor                                         #  
#          - Adafruit Bluefruit EZ-Link                                 #
#          - NeoPixel Stick - 8 x 5050 RGB LED with Integrated Drivers  #
#          - SD CARD SLOT                                               #
#          - etc..                                                      #
#########################################################################

# Development environment - Arduino IDE

# Using Library - SDfat.h
# Using OpenSource code - Neopixel_Painter

# function - The image is transmitted via Bluetooth. 
#          - reads the mapped image and blinks the LED light according to the pixel.
#          - The speed of one wheel is measured using a hall sensor.

##############################
# S/W -> Android application #
##############################
# Development environment - Android Studio

# function - After selecting and shooting an image, edit it and convert it to POV image data and save it.
#          - The application transfers the converted image to the SD card inserted in the SD card slot,
#            and receives the data in bytes.

