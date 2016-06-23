# Atom
Simple web ui/embedded IoT project using python and twisted.
It can be an example for a small embedded project where web UI is needed.

In this example web UI is used to control small two wheeled rover with AVR328p micro in place.

Rover can perform following:

1) It can move one wheel or both wheels (left, right, forward commands)

2) It can store sensors data in EEPROM and send it later by a separate command (read from eeprom)


# USAGE
mc - folder contains example implementation for an avr328p board with GY-88 and TB6612FNG driver curcuit.

atom - folder contains classes for serial, websockets and web interfaces.

main.py - main script to run via twistd (eg. twistd -n web --class=main.resource -p 8000)

# INSTALLATION
Make sure you have python >= 3.5 and required python libraries (project is not packaged up till now)
