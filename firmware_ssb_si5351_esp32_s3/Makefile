.DEFAULT_GOAL := build

IDF_IMAGE ?= espressif/idf:latest
PROJECT_DIR := $(CURDIR)
BUILD_DIR := $(PROJECT_DIR)/build
OUTPUT_DIR := $(PROJECT_DIR)/build-output
PROJECT_NAME := polar_ssb_si5351_esp32s3

.PHONY: build docker local-build local-menuconfig flash monitor clean

build docker:
	mkdir -p "$(OUTPUT_DIR)"
	docker run --rm \
		-v "$(PROJECT_DIR):/project" \
		-w /project \
		"$(IDF_IMAGE)" \
		idf.py build
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
	# idf.py -p $${PORT:-/dev/ttyACM0} flash
	cd build-output && python3 -m esptool \
	    --chip esp32s3 \
	    -p /dev/tty.usbmodem11301 \
	    -b 460800 \
	    --before default-reset \
	    --after hard-reset \
	    write-flash \
	    --flash-mode dio \
	    --flash-freq 80m \
	    --flash-size 2MB \
	    0x0 bootloader.bin \
	    0x8000 partition-table.bin \
	    0x10000 polar_ssb_si5351_esp32s3.bin

monitor:
	idf.py -p $${PORT:-/dev/ttyACM0} monitor

clean:
	docker run --rm \
		-v "$(PROJECT_DIR):/project" \
		-w /project \
		"$(IDF_IMAGE)" \
		idf.py fullclean
