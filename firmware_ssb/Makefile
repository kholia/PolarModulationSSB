stuff:
	mkdir -p build-output
	docker build . -t firmware_ssb:latest
	docker compose up

flash:
	# ./picoload.sh build-output/usb_analog_microphone.uf2
	picotool load -f build-output/usb_analog_microphone.uf2


stuff-alt:
	docker build -f Dockerfile.Fedora .
