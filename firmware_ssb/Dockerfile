# Dockerfile for building Raspberry Pi Pico RP2350 firmware
# Based on Ubuntu latest 'devel' - https://hub.docker.com/_/ubuntu

FROM ubuntu:devel

# Build argument for serial number binding (optional)
ARG AUTHORIZED_SERIAL=""

# Prevent interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set the Pico board type
ENV PICO_BOARD=waveshare_rp2350_zero

# Set PICO_SDK_PATH to the location where it will be in the container
ENV PICO_SDK_PATH=/workspace/pico-sdk

# Install required dependencies for Pico SDK and CMake builds
RUN apt-get update && apt-get install -y \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    libstdc++-arm-none-eabi-newlib \
    git \
    python3 \
    python3-pip \
    build-essential pkg-config libusb-1.0-0-dev cmake \
    && rm -rf /var/lib/apt/lists/*

# Create workspace directory
WORKDIR /workspace

# Clone the Pico SDK from the develop branch
RUN git clone --branch develop https://github.com/raspberrypi/pico-sdk.git /workspace/pico-sdk && \
    cd /workspace/pico-sdk && \
    git submodule update --init --recursive

RUN cd /workspace/pico-sdk && \
    git pull && \
    cd lib/tinyusb && \
    git checkout master && git pull

# Install picotool for flashing Pico devices
RUN git clone https://github.com/raspberrypi/picotool.git /tmp/picotool && \
    cd /tmp/picotool && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install && \
    cd / && \
    rm -rf /tmp/picotool

ADD "https://www.random.org/cgi-bin/randbyte?nbytes=10&format=h" skipcache

# Copy the firmware source code
COPY . /workspace/

# Build the firmware with optional serial number binding
RUN chmod +x build.sh && \
    if [ -n "$AUTHORIZED_SERIAL" ]; then \
        echo "Building with authorized serial: $AUTHORIZED_SERIAL" && \
        cmake -B build -S . -DAUTHORIZED_SERIAL="$AUTHORIZED_SERIAL"; \
    else \
        echo "Building DEMO firmware (no serial restriction)" && \
        cmake -B build -S .; \
    fi && \
    cmake --build build -j$(nproc) && \
    echo "Build complete. Looking for artifacts..." && \
    find build -name '*.uf2' -o -name '*.elf' -o -name '*.bin' -o -name '*.hex' | head -10

# Default command - show build artifacts
CMD ["sh", "-c", "echo 'Build complete. Firmware artifacts:' && ls -lh *.uf2 *.elf *.bin 2>/dev/null || echo 'No firmware files found in root directory'"]
