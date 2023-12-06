import carla
import random
import time
client = carla.Client('localhost', 2000)
world = client.get_world()
# client.load_world('Town04_Opt')
client.load_world('Town10HD_Opt')
world.unload_map_layer(carla.MapLayer.Walls) # Causes crash when you try to re-run script with existing CARLA application
# world.unload_map_layer(carla.MapLayer.Foliage) # Causes crash when you try to re-run script with existing CARLA application
spectator = world.get_spectator()
spectator.set_transform(carla.Transform(carla.Location(x=-227.037689, y=162.543137, z=8.654385), carla.Rotation(pitch=1.956870, yaw=-1.042053, roll=0.000042)))
while True:
    print(spectator.get_transform())
    print(spectator.get_transform().location)
    time.sleep(1)

