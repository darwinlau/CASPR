--- Fixing /dev/ on Linux ---

Plug the VESC into a desired USB port.

CLI: $ ls /dev 

to find the new ttyACMx port.

Suppose the device is /dev/ttyACM0: 

CLI: $ udevadm info --name=/dev/ttyACM0 --attribute-walk

In the second entry, mark down KERNEL and SUBSYSTEMS fields.

e.g. KERNELS=="1-1.1.2", SUBSYSTEMS=="usb"

CLI: $ sudo nano /etc/udev/rules.d/99-USB_VESC.rules

add 
KERNELS=="1-1.1.2", SUBSYSTEMS=="usb", SYMLINK+="VESC_001"

where "VESC_001" is the fixed desired USB device name.

When choosing a SYMLINK name with numbers, always prefix them with the 
right amount of 0's instead of inls cluding just the number. 

E.g. VESC_001 instead of VESC_1

Add more lines of KERNELS, SUBSYSTEMS, SYMLINK as you desire.

Save, exit, reboot.

"VESC_1" is now fixed to that particular USB port.

Use /dev/VESC_001 for future communications.