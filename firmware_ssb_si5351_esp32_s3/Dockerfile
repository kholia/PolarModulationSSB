ARG IDF_IMAGE=espressif/idf:latest
FROM ${IDF_IMAGE}

WORKDIR /project
COPY . /project
RUN /bin/bash -lc 'source /opt/esp/idf/export.sh && idf.py build'

CMD ["/bin/bash", "-lc", "cp -v build/polar_ssb_si5351_esp32s3.bin build/polar_ssb_si5351_esp32s3.elf build/bootloader/bootloader.bin build/partition_table/partition-table.bin build/flash_args build-output/"]
