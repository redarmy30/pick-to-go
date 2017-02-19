#import win32com.client
import sys
import usb
import usb.core
import usb.util

#wmi = win32com.client.GetObject ("winmgmts:")
#for usb in wmi.InstancesOf ("Win32_USBHub"):
#    print usb.DeviceID



# decimal vendor and product values
dev = usb.core.find(find_all = 1)
