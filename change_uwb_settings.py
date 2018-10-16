from pypozyx import *

pozyx = PozyxSerial(get_first_pozyx_serial_port())

current_uwb_settings = UWBSettings()
pozyx.getUWBSettings(current_uwb_settings)

print("Currently on UWB settings {}".format(current_uwb_settings))

new_uwb_settings = current_uwb_settings
new_uwb_settings.channel = 2    # default 5
new_uwb_settings.bitrate = 2    # default 0
new_uwb_settings.plen = 0x08    # default 0x08
new_uwb_settings.gain_db = 24   # default 11.5

devices = [0x6746, 0x6747, 0x674b, 0x6764, 0x6756, 0x6727, 0x6e19, 0x6767, 0x6e17]

for device in devices:
    status = pozyx.setUWBSettings(new_uwb_settings, device)
    if status == POZYX_SUCCESS:
        print("Set UWB settings on ID {}".format(hex(device)))
    else:
        print("Failed to set UWB settings on ID {}".format(hex(device)))
status = pozyx.setUWBSettings(new_uwb_settings)
if status == POZYX_SUCCESS:
    print("Set UWB settings on local device")

## Uncomment to save settings on devices.
# for device in devices:
#   status = pozyx.saveUWBSettings(device)
#   if status == POZYX_SUCCESS:
#     print("Saved UWB settings on ID {}".format(hex(device)))
#   else:
#     print("Failed to save UWB settings on ID {}".format(hex(device)))
#
# pozyx.saveUWBSettings()