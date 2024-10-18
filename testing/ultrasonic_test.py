import gpiozero
import time

sensor = gpiozero.DistanceSensor(echo=18,trigger=17)

for j in range(10):
    print('Distance: ', sensor.distance * 100)
    time.sleep(1)
