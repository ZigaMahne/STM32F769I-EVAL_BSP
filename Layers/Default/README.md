# Board: STMicroelectronics [STM32F769I-EVAL](https://www.st.com/en/evaluation-tools/stm32f769i-eval.html)

## Default Board Layer

Device: **STM32F769NIHx**

System Core Clock: **216 MHz**

This setup is configured using **STM32CubeMX**, an interactive tool provided by STMicroelectronics for device configuration.
Refer to ["Configure STM32 Devices with CubeMX"](https://github.com/Open-CMSIS-Pack/cmsis-toolbox/blob/main/docs/CubeMX.md) for additional information.

### System Configuration

| System resource       | Setting
|:----------------------|:--------------------------------------
| Heap                  | 64 kB (configured in the STM32CubeMX)
| Stack (MSP)           |  1 kB (configured in the STM32CubeMX)

### STDIO mapping

**STDIO** is routed to Virtual COM port on the ST-LINK (using **USART1** peripheral)

### CMSIS-Driver mapping

| CMSIS-Driver          | Peripheral            | Board connector/component                             | Connection
|:----------------------|:----------------------|:------------------------------------------------------|:------------------------------
| Driver_ETH_MAC0       | ETH                   | Ethernet RJ45 connector (CN9)                         | CMSIS_ETH
| Driver_ETH_PHY0       | DP83848C (external)   | Ethernet RJ45 connector (CN9)                         | CMSIS_ETH
| Driver_MCI1           | SDMMC1                | MicroSD card slot (CN17)                              | CMSIS_MCI
| Driver_USART1         | USART1                | ST-LINK connector (CN22)                              | STDIN, STDOUT, STDERR
| Driver_USBD0          | USB_OTG_FS            | USB OTG1 FS connector (CN13)                          | CMSIS_USB_Device
| Driver_USBH1          | USB_OTG_HS            | USB OTG2 HS connector (CN8)                           | CMSIS_USB_Host
| CMSIS-Driver VIO      | GPIO                  | LEDs (LD3, LD1, LD4, LD2) and Tamper button (B2)      | CMSIS_VIO

### CMSIS-Driver Virtual I/O mapping

| CMSIS-Driver VIO      | Board component
|:----------------------|:--------------------------------------
| vioBUTTON0            | Tamper button (B2)
| vioLED0               | LED red       (LD3)
| vioLED1               | LED green     (LD1)
| vioLED2               | LED blue      (LD4)
| vioLED3               | LED orange    (LD2)
