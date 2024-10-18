import time
import board
import adafruit_tcs34725

# Initialise the I2C connection with the sensor
i2c = board.I2C()
sensor = adafruit_tcs34725.TCS34725(i2c)

#def normalise_rgb(rgb):
#    return (rgb[0]/255,rgb[1]/255,rgb[2]/255)

def adjust_for_lux(rgb, lux, reference_lux=1000):
    factor = lux / reference_lux

    return (rgb[0] * factor, rgb[1] * factor, rgb[2] * factor)

def white_balance_adjustment(rgb, color_temp):
    correction_factors = {
        2700: (1.2, 1.1, 1.0),
        4000: (1.1, 1.0, 0.9),
        6500: (1.0, 1.0, 1.0),
        10000: (0.9, 0.9, 1.0)
    }
    r_factor, g_factor, b_factor = correction_factors.get(color_temp, (1.0, 1.0, 1.0))
    return (rgb[0] * r_factor, rgb[1] * g_factor, rgb[2] * b_factor)

def get_real_color(rgb, lux, color_temp):
    #rgb_normalised = normalise_rgb(rgb)
    rgb_adjusted = adjust_for_lux(rgb, lux)
    rgb_corrected = white_balance_adjustment(rgb_adjusted, color_temp)
    return rgb_corrected

while True:
    # Read in values from TCS34725 RGB sensor
    rgb = sensor.color_rgb_bytes  # RGB values
    lux = sensor.lux              # Example lux value
    color_temp = sensor.color_temperature      # Example color temperature in Kelvin

    print(f"Sensor colour: {rgb}")
    adj_color = adjust_for_lux(rgb, lux)
    print(f"Colour after lux adjustment: {adj_color}")
    white_color = white_balance_adjustment(adj_color, color_temp)
    print(f"Colour after temp adjustment: {white_color}")
    #real_color = get_real_color(rgb, lux, color_temp)

    #print(f"Real Color: {real_color}")
    time.sleep(3.0)

# Sensor behaviour properties
sensor.integration_time = 200
sensor.gain = 60

# TODO: when interrupt triggers, save coordinates as defined boundary
