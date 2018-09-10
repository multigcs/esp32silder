

all:
	platformio run

flash:
	platformio run -v --target upload

monitor:
	gtkterm --port /dev/ttyUSB0 --speed 115200



