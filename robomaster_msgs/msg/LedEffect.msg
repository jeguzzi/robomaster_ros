uint8 BOTTOM_BACK = 1
uint8 BOTTOM_FRONT = 2
uint8 BOTTOM_LEFT = 4
uint8 BOTTOM_RIGHT = 8
uint8 TOP_LEFT = 16
uint8 TOP_RIGHT = 32

uint8 OFF = 0
uint8 ON = 1
uint8 BREATH = 2
uint8 FLASH = 3
uint8 SCROLLING = 4
uint8 PULSE = 5

uint8 mask 63
uint8 effect 1
std_msgs/ColorRGBA color
float32 t1 1.0
float32 t2 1.0
# bool loop true
