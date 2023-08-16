# ENC28J60-Mbed-EMAC-Driver

This driver is an adaptation of the existing ENC28J60 EMAC driver from **Zoltan Hudak**
(see [https://os.mbed.com/users/hudakz/code/ENC28J60-EMAC/](https://os.mbed.com/users/hudakz/code/ENC28J60-EMAC/))
which is based on the original work from **tobiasjaster** (see [https://github.com/tobiasjaster/ENC28J60-EMAC-Driver](https://github.com/tobiasjaster/ENC28J60-EMAC-Driver)).

This driver is working on **Mbed OS 6**. RX filters have been removed to allow using this driver for non-endpoint nodes.
This allows project such as [Nanostack border router](https://github.com/PelionIoT/nanostack-border-router) to work
on microcontrollers that don't have an embedded Ethernet MAC peripheral with help of the external ENC28J60 Ethernet MAC.

# Usage

 - Simply clone this repo into your Mbed OS project, or add it as a library (.lib file) using the standard method.
 - Configure the driver in `mbed_app.json` by override the following parameters:
   <pre>"enc28j60-emac.sck": "ETH_SCLK",
   "enc28j60-emac.cs": "ETH_CSn",
   "enc28j60-emac.miso": "ETH_MISO",
   "enc28j60-emac.mosi": "ETH_MOSI",</pre>
 - Optionally adjust the SPI frequency to your requirements using `"enc28j60-emac.spi-freq": <custom_freq>` in your `mbed_app.json`.
