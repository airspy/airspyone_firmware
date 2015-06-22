@echo off
echo Downloading algo...
dfu-util -d 1fc9:000c -t 2048 -R -D iram_dfu_util_spiflash.bin.hdr
echo Programming ../airspy_rom_to_ram_packing.bin
lpcdfu -d 3 -e -D ../airspy_rom_to_ram_packing.bin -U
Pause
