#!/bin/bash
set -e

#----------------------
# Set camera settings |
#----------------------

# Set brightness (int): min=30 max=255 step=1 default=133
v4l2-ctl -c brightness=133

# Set contrast (int): min=0 max=10 step=1 default=5
v4l2-ctl -c contrast=5

# Set saturation (int): min=0 max=200 step=1 default=83
v4l2-ctl -c saturation=83

# Set  white_balance_temperature_auto (bool): default=1
v4l2-ctl -c white_balance_temperature_auto=0

# Set  power_line_frequency (menu): min=0 max=2 default=2
#				0: Disabled
#				1: 50 Hz
#				2: 60 Hz
v4l2-ctl -c power_line_frequency=0

# Set white_balance_temperature (int): min=2800 max=10000 step=1 default=4500
v4l2-ctl -c white_balance_temperature=4500

# Set sharpness (int): min=0 max=50 step=1 default=25
v4l2-ctl -c sharpness=25

# Set backlight_compensation (int): min=0 max=10 step=1 default=0
v4l2-ctl -c backlight_compensation=0

# Set exposure_auto (menu): min=0 max=3 default=1
#				1: Manual Mode
#				3: Aperture Priority Mode
v4l2-ctl -c exposure_auto=1

# Set exposure_absolute (int): min=5 max=20000 step=1 default=156
v4l2-ctl -c exposure_absolute=5

# Set pan_absolute (int): min=-201600 max=201600 step=3600 default=0
v4l2-ctl -c pan_absolute=0

# Set tilt_absolute (int): min=-201600 max=201600 step=3600 default=0
v4l2-ctl -c tilt_absolute=0

# Set focus_auto (bool): default=0
v4l2-ctl -c focus_auto=0

# Set  focus_absolute (int): min=0 max=40 step=1 default=0
v4l2-ctl -c focus_absolute=36

# Set zoom_absolute (int): min=0 max=10 step=1 default=0
v4l2-ctl -c zoom_absolute=0

# Display results
echo
v4l2-ctl --list-ctrls-menus

# Set video format 
v4l2-ctl --set-fmt-video width=640,height=480,pixelformat=0

# Display video format
echo
v4l2-ctl -V

