default:  monitor

build:
	platformio run

upload-debug:
	platformio run --environment uno-debug --target upload

upload:
	platformio run --target upload

debug: upload-debug
	platformio device monitor

monitor: upload
	platformio device monitor
