| Supported Targets | ESP32-H2 | ESP32-C6 |
| ----------------- | -------- | -------- |

# Window Covering Example

This test code shows how to configure Zigbee end-device and use it as an HA thermostat.

## Hardware Required

* A USB cable for power supply and programming
* Choose a ESP32-H2 as Zigbee end-device (see [HA_window_covering](../HA_window_covering/))

## Configure the project

Before project configuration and build, make sure to set the correct chip target using `idf.py set-target TARGET` command.

## Erase the NVRAM

Before flash it to the board, it is recommended to erase NVRAM if user doesn't want to keep the previous examples or other projects stored info
using `idf.py -p PORT erase-flash`

## Build and Flash

Build the project, flash it to the board, and start the monitor tool to view the serial output by running `idf.py -p PORT flash monitor`.

(To exit the serial monitor, type ``Ctrl-T + X``.)

## Example Output

As you run the example, you will see the following log:

```
I (23) boot: ESP-IDF v5.1.3-dirty 2nd stage bootloader
I (24) boot: compile time Jun 30 2024 19:29:10
I (25) boot: chip revision: v0.1
I (27) boot.esp32h2: SPI Speed      : 64MHz
I (32) boot.esp32h2: SPI Mode       : DIO
I (37) boot.esp32h2: SPI Flash Size : 2MB
I (41) boot: Enabling RNG early entropy source...
I (47) boot: Partition Table:
I (50) boot: ## Label            Usage          Type ST Offset   Length
I (58) boot:  0 nvs              WiFi data        01 02 00009000 00006000
I (65) boot:  1 phy_init         RF data          01 01 0000f000 00001000
I (73) boot:  2 factory          factory app      00 00 00010000 000e1000
I (80) boot:  3 zb_storage       Unknown data     01 81 000f1000 00004000
I (87) boot:  4 zb_fct           Unknown data     01 81 000f5000 00000400
I (95) boot: End of partition table
I (99) esp_image: segment 0: paddr=00010020 vaddr=42078020 size=10440h ( 66624) map
I (128) esp_image: segment 1: paddr=00020468 vaddr=40800000 size=07bb0h ( 31664) load
I (140) esp_image: segment 2: paddr=00028020 vaddr=42000020 size=7322ch (471596) map
I (284) esp_image: segment 3: paddr=0009b254 vaddr=40807bb0 size=0489ch ( 18588) load
I (292) esp_image: segment 4: paddr=0009faf8 vaddr=4080c450 size=01600h (  5632) load
I (298) boot: Loaded app from partition at offset 0x10000
I (299) boot: Disabling RNG early entropy source...
I (313) cpu_start: Unicore app
I (314) cpu_start: Pro cpu up.
W (323) clk: esp_perip_clk_init() has not been implemented yet
I (329) cpu_start: Pro cpu start user code
I (330) cpu_start: cpu freq: 96000000 Hz
I (330) cpu_start: Application information:
I (332) cpu_start: Project name:     window_covering
I (338) cpu_start: App version:      044fc28-dirty
I (343) cpu_start: Compile time:     Jun 30 2024 19:29:05
I (349) cpu_start: ELF file SHA256:  30fc842d0985d52f...
I (355) cpu_start: ESP-IDF:          v5.1.3-dirty
I (361) cpu_start: Min chip rev:     v0.0
I (365) cpu_start: Max chip rev:     v0.99 
I (370) cpu_start: Chip rev:         v0.1
I (375) heap_init: Initializing. RAM available for dynamic allocation:
I (382) heap_init: At 40812D10 len 0003A670 (233 KiB): D/IRAM
I (389) heap_init: At 4084D380 len 00002B60 (10 KiB): STACK/DIRAM
I (396) spi_flash: detected chip: generic
I (400) spi_flash: flash io: dio
W (404) spi_flash: Detected size(4096k) larger than the size in the binary image header(2048k). Using the size in the binary image header.
I (417) sleep: Configure to isolate all GPIO pins in sleep state
I (424) sleep: Enable automatic switching of GPIO sleep configuration
I (431) app_start: Starting scheduler on CPU0
I (436) main_task: Started on CPU0
I (436) main_task: Calling app_main()
I (456) phy: phy_version: 230,2, 9aae6ea, Jan 15 2024, 11:17:12
I (456) phy: libbtbb version: 944f18e, Jan 15 2024, 11:17:25
I (466) main_task: Returned from app_main()
I (596) ESP_ZB_WINDOW_COVERING: ZDO signal: ZDO Config Ready (0x17), status: ESP_FAIL
I (596) ESP_ZB_WINDOW_COVERING: Initialize Zigbee stack
I (606) ESP_ZB_WINDOW_COVERING: Deferred driver initialization successful
I (606) ESP_ZB_WINDOW_COVERING: Device started up in  factory-reset mode
I (616) ESP_ZB_WINDOW_COVERING: Start network steering
I (2866) ESP_ZB_WINDOW_COVERING: Network steering was not successful (status: ESP_FAIL)
I (6126) ESP_ZB_WINDOW_COVERING: Network steering was not successful (status: ESP_FAIL)
I (9386) ESP_ZB_WINDOW_COVERING: Network steering was not successful (status: ESP_FAIL)
I (12646) ESP_ZB_WINDOW_COVERING: Network steering was not successful (status: ESP_FAIL)
```

## Window Covering Functions


## Troubleshooting

For any technical queries, please open an [issue](https://github.com/espressif/esp-zigbee-sdk/issues) on GitHub. We will get back to you soon.
