

all:
	platformio run

flash:
	platformio run -v --target upload

monitor:
	gtkterm --speed 115200 --port /dev/ttyUSB0 
