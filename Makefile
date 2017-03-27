default:  monitor

build:
	platformio run

upload-wifi:
	platformio run --environment uno-wifi --target upload

upload:
	platformio run --target upload

wifi: upload-wifi
	platformio device monitor

monitor: upload
	platformio device monitor
