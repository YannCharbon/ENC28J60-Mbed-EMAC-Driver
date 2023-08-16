# ENC28J60-Mbed-EMAC-Driver

This driver is an adaptation of the existing ENC28J60 EMAC driver from **Zoltan Hudak**
(see [https://os.mbed.com/users/hudakz/code/ENC28J60-EMAC/](https://os.mbed.com/users/hudakz/code/ENC28J60-EMAC/))
which is based on the original work from **tobiasjaster** (see [https://github.com/tobiasjaster/ENC28J60-EMAC-Driver](https://github.com/tobiasjaster/ENC28J60-EMAC-Driver)).

This driver is working on **Mbed OS 6**. RX filters have been removed to allow using this driver for non-endpoint nodes.
This allows project such as [Nanostack border router](https://github.com/PelionIoT/nanostack-border-router) to work
on microcontrollers that don't have an embedded Ethernet MAC peripheral with help of the external ENC28J60 Ethernet MAC.

# Usage

 - Connect the `MOSI`, `MISO`, `SCK` and `CS` pins of the ENC28J60 to your board. The `INT` and `RESET` pins are not used.
 - Simply clone this repo into your Mbed OS project, or add it as a library (.lib file) using the standard method.
 - Configure the driver in `mbed_app.json` by override the following parameters with your own values:
   <pre>"enc28j60-emac.sck": "ETH_SCLK",
   "enc28j60-emac.cs": "ETH_CSn",
   "enc28j60-emac.miso": "ETH_MISO",
   "enc28j60-emac.mosi": "ETH_MOSI",</pre>
 - Optionally adjust the SPI frequency to your requirements using `"enc28j60-emac.spi-freq": <custom_freq>` in your `mbed_app.json`.
 - To set the MAC address define an array with the desired address bytes and call the `set_hwaddr(mac)` function before calling the network interface `connect` function (see following example).
    ```
    const uint8_t       MAC[6] = { 0, 1, 2, 3, 4, 5 };
    EthernetInterface   net;

    int main()
    {
        net.get_emac().set_hwaddr(MAC);             // set MAC address
        if (net.connect() != 0) {
            printf("Error: Unable to connect to the network.\n");
            return -1;
        }
    ...
    }
    ```
  - If you want to use this driver with the Nanostack border router project, you have simply to set `"app.backhaul-mac-src": "CONFIG"` and override `"app.backhaul-mac": "{ 0x1, 0x2, 0x3, 0x4, 0x5, 0x6 }"` in the `mbed_app.json`. In this case, the following step must be ignored. Other existing networking projects might require different steps.
