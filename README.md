BEETHEFIRST-bootloader by ![alt text](https://www.beeverycreative.com/client/skins/images/logo.png "Logo Title Text 1")
===============================

Here you find the sources of the bootloader for printers based on the BEETHEFIRST 3D printer. The bootloader has a GPL license so you can use it and edit it. It is a mashup between many original parts and you can find the respective acknowledges in the sources.

Please find the stable releases here: https://github.com/beeverycreative/beethefirst-bootloader/tree/develop/releases

You may flash the bootloader using the [BootloaderUpdaer](https://github.com/beeverycreative/BootloaderUpdater) tool.

Development
-------- 
If you are interested in following and/or participating on development, feel free to contact us to: jpinto@beeverycreative.com

Tools
------------
The ARM toolchain used is [Sourcery G++ Lite 2010q1-188 for ARM EABI](https://sourcery.mentor.com/sgpp/lite/arm/portal/release1294).

Building the firmware 
------------
On the bootloader sources folder you should execute make all to build the bootloader. The built bootloader will be placed inside FLASH_RUN folder. You may after write the bootloader using the [BootloaderUpdaer](https://github.com/beeverycreative/BootloaderUpdater) tool or by using an In-System Programming tool such as [LPC21ISP](http://lpc21isp.sourceforge.net/).


