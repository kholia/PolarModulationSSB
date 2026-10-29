.DEFAULT_GOAL := build

IDF_IMAGE ?= espressif/idf:latest
PROJECT_DIR := $(CURDIR)
BUILD_DIR := $(PROJECT_DIR)/build
OUTPUT_DIR := $(PROJECT_DIR)/build-output
PROJECT_NAME := polar_ssb_si5351_esp32s3
# Serial port of the ESP32-S3 USB Serial/JTAG. Defaults to the first macOS
# usbmodem device; override with e.g. `make flash PORT=/dev/ttyACM0`.
PORT ?= $(firstword $(wildcard /dev/cu.usbmodem*) /dev/ttyACM0)
BAUD ?= 460800
ESPTOOL ?= $(shell command -v esptool >/dev/null 2>&1 && echo esptool || echo python3 -m esptool)

.PHONY: build docker local-build local-menuconfig flash build-flash monitor clean

build docker:
	mkdir -p "$(OUTPUT_DIR)"
	docker run --rm \
		-v "$(PROJECT_DIR):/project" \
		-w /project \
		"$(IDF_IMAGE)" \
		bash -lc 'idf.py build || { echo "Build failed; retrying after idf.py fullclean (stale build/ from another IDF image or Python env)"; idf.py fullclean && idf.py build; }'
	cp -f "$(BUILD_DIR)/$(PROJECT_NAME).bin" "$(OUTPUT_DIR)/"
	cp -f "$(BUILD_DIR)/$(PROJECT_NAME).elf" "$(OUTPUT_DIR)/"
	cp -f "$(BUILD_DIR)/bootloader/bootloader.bin" "$(OUTPUT_DIR)/"
	cp -f "$(BUILD_DIR)/partition_table/partition-table.bin" "$(OUTPUT_DIR)/"
	cp -f "$(BUILD_DIR)/flash_args" "$(OUTPUT_DIR)/"

local-build:
	idf.py build

local-menuconfig:
	idf.py menuconfig

flash:
	cd build-output && $(ESPTOOL) \
	    --chip esp32s3 \
	    -p $(PORT) \
	    -b $(BAUD) \
	    --before default-reset \
	    --after hard-reset \
	    write-flash \
	    --flash-mode dio \
	    --flash-freq 80m \
	    --flash-size 2MB \
	    0x0 bootloader.bin \
	    0x8000 partition-table.bin \
	    0x10000 $(PROJECT_NAME).bin

build-flash: build
	$(MAKE) flash

monitor:
	idf.py -p $${PORT:-/dev/ttyACM0} monitor

clean:
	docker run --rm \
		-v "$(PROJECT_DIR):/project" \
		-w /project \
		"$(IDF_IMAGE)" \
		idf.py fullclean
