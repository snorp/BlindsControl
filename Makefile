SERIAL_PORT ?= /dev/cu.usbserial-DN00Z3UY

ENV ?= debug

all: build

clean:
	platformio run -e $(ENV) -t clean

build:
	platformio run -e $(ENV)

install:
	platformio run -e $(ENV) -t upload --upload-port $(SERIAL_PORT)

monitor:
	platformio serialports monitor --port $(SERIAL_PORT)
