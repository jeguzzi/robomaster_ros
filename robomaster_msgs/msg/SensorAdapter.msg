# Summarizes the state of up to 6 sensor adapters (each with 2 interfaces)

std_msgs/Header header   # header with the time stamp of the update
uint8[12] io             # input pin state (0 or 1)
int16[12] adc            # adc state
uint8[12] port [1, 2, 1, 2, 1, 2, 1, 2, 1, 2, 1, 2]  # interface intentifier
uint8[12] id [1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6]    # sensor adapter identifier
