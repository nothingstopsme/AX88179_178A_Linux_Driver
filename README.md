# AX88179_178A_Linux_Driver
This is a revised version of AX88179_178A linux driver based on the source code [(v1.20.0)](https://www.asix.com.tw/en/support/download/file/120) provided by the manufacturer.

## Disclaimer
I am not an expert in electronic engineering or linux driver development, and all the modifications and tips offered here are simply derived from my trial-and-error experiments without the knowledge of and the access to the detailed device specification. So while I believe these changes could help others trying to have this type of network adaptor work, please do check if they are really suitable for your case and apply them at your own risk.

## Changes
1. Rewriting the ax88179_rx_fixup() so that it will not trigger its parent driver (usbnet) to record fake rx errors, which cause high error rates and lead to the interruption of packet receiving.
2. Rewriting the ax88179_tx_fixup() so that when the size of a packet to be sent is a multiple of the maximum packet size defined for USB bulk transfer, the padding mechanism for fulfilling the requirement of USB standard would not lead to a malfunction of the device.
3. Updating the condition for the use of the feature of "hardware IP alignment", so that the driver can actually benefit from that hardware acceleration if the required alignment happens to be supported by the device.

## Extra Tips
During my tests it has also been found that this hardware seems quite susceptible to signal noise, especially when running at super speed mode (usb 3.0). For example, when an additional 1-metre-long usb extention cable (which had worked well with my previous network adaptor) was connected between my device and a host socket, a few second's to a few minute's worth of packet transmission at super speed would cause malfunctions of it and no subsequent packets could be sent or received (albeit the corresponding network interface was still reported by the ifconfig command); on the other hand, this device could survive the following two loading tests:
* the same host + the same extention cable + at high speed (usb 2.0)
* the same host + the same extention cable + at super speed + replacing the attached cable to the device with one of good shielding (Yes, I did resoldering)

In my view those results indicate that the noise in high frequency signal received by the device could easily damage its stability, and one might reduce this impact via:
* Plugging it directly to a host socket, or
* Using a quality extension cable (as short as possible) if you really need it.

