import carla
import random
import time
client = carla.Client('localhost', 2000)
world = client.get_world()
client.load_world('Town05')
spectator = world.get_spectator()
while True:
    print(spectator.get_transform())
    print(spectator.get_transform().location)
    time.sleep(1)

