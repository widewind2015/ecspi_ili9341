# ecspi_ili9341
LVGL for SPI LCD ILI9341 on Verdin iMX8M Plus M7 Core.

Test with SDK_2_12_1_MIMX8ML8xxxKZ from NXP.

Check out at `SDK_2_12_1_MIMX8ML8xxxKZ/boards/evkmimx8mp/rtos_examples/freertos_ecspi/`.

Build the demo with `./build_ddr_release.sh` as LVGL needs large RAM more than on-chip TCM. The demo will run on the DDR RAM of Verdin iMX8M Plus.

`m7bootddr` is defined in U-Boot. Fetch M7 firmware from a TFTP server and load it to `0x80000000` which is at DDR RAM.
```
Verdin iMX8MP # print m7bootddr
m7bootddr=tftp 0x80000000 m7.bin; dcache flush; bootaux 0x80000000
```

[![](https://telegraph-image-cqn.pages.dev/file/7024547aacc2ce5184190.png)](https://player.bilibili.com/player.html?aid=785775492&bvid=BV1114y1R7zy&cid=1194065571&page=1)
