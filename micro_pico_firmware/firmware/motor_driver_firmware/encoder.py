from rotary_irq_rp2 import RotaryIRQ

# Pins for encoder signal A and B
#rotary = RotaryIRQ(14, 15)

rotary_right = RotaryIRQ(14, 15, pull_up=True)
rotary_left = RotaryIRQ(12, 13, pull_up=True)

print("initializing encoder with 48 CPR")


def read_velocity():
    velocity_l = rotary_left.velocity()
    velocity_r = rotary_right.velocity()
    #print("current velocity is: ", velocity_l, " ", velocity_r)
    return velocity_l, velocity_r

def read_encoder():
    encoder_l = rotary_left.value()
    encoder_r = rotary_right.value()
    #print("current encoder val:  Right:  ", encoder_r, "Left:  ", encoder_l)
    return encoder_l, encoder_r
