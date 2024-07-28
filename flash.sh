esptool --chip esp32c3 --baud 460800 --before default_reset --after hard_reset write_flash -z --flash_mode dio --flash_freq 80m --flash_size 4MB \
    0x0000 /home/victor/coding/projects/asac-joystick/.pio/build/esp32-c3/bootloader.bin \
    0x8000 /home/victor/coding/projects/asac-joystick/.pio/build/esp32-c3/partitions.bin \
    0xe000 /home/victor/.platformio/packages/framework-arduinoespressif32/tools/partitions/boot_app0.bin \
    0x10000 .pio/build/esp32-c3/firmware.bin