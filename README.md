# Atom
Simple web ui/embedded IoT project using python and twisted.
It can be an example for a small embedded project where web UI is needed.


# USAGE
mc - folder contains example implementation for an avr328p board with GY-88 and TB6612FNG driver curcuit.

atom - folder contains classes for serial, websockets and web interfaces.

main.py - main script to run via twistd (eg. twistd -n web --class=main.resource -p 8000)

# INSTALLATION
Make sure you have python >= 3.5 and required python libraries (project is not packaged up till now)
