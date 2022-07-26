# Control commands for all LEDs

uint8 BOTTOM_BACK = 1     # mask: chassis rear LED
uint8 BOTTOM_FRONT = 2    # mask: chassis front LED
uint8 BOTTOM_LEFT = 4     # mask: chassis left LED
uint8 BOTTOM_RIGHT = 8    # mask: chassis right LED
uint8 BOTTOM = 15         # mask: all chassis LEDs
uint8 TOP_LEFT = 16       # mask: gimbal left LED
uint8 TOP_RIGHT = 32      # mask: gimbal right LED
uint8 TOP = 48            # mask: all gimbal LEDs
uint8 ALL = 63            # mask: all LEDs

uint8 OFF = 0             # effect: switch off LED
uint8 ON = 1              # effect: switch LED to solid color
uint8 BREATH = 2          # effect: breath
uint8 FLASH = 3           # effect: flash LED
uint8 SCROLLING = 4       # effect: scrolling
uint8 PULSE = 5           # effect: pulse LED

uint8 mask 63             # bitmask mask to select which LEDs to control

uint8 submask 255         # bitmask to select which portions of the gimbal LED to control.
                          # The 7 sub LEDs are enumerate clockwise
uint8 effect 1            # One of the effect enums
std_msgs/ColorRGBA color  # The desired LED color (not relevant for ``effect=OFF``)
float32 t1 1.0            # On interval duration, only relevant for effects ``PULSE`` and ``FLASH``.
float32 t2 1.0            # Off interval duration, only relevant for effects ``PULSE`` and ``FLASH``.
