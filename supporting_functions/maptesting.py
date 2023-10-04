import numpy as np
from PIL import Image
import carla
import random
from agents.navigation.basic_agent import BasicAgent
import time

# sources
# https://carla.readthedocs.io/en/0.9.14/core_map/#carla-maps
# https://carla.readthedocs.io/en/0.9.14/tuto_M_custom_layers/
# https://carla.readthedocs.io/en/latest/core_map/#changing-the-map

# Carla world generation
client = carla.Client('localhost', 2000)
world = client.get_world()
client.load_world('Town02_Opt') # Layered map https://carla.readthedocs.io/en/0.9.14/core_map/#carla-maps
world.unload_map_layer(carla.MapLayer.All) # Causes crash when you try to re-run script with existing CARLA application

# client.load_world('Town04')
# client.load_world('Town10HD')

# spawn vehicle
vehicle_blueprints = world.get_blueprint_library().filter('vehicle.tesla.model3')
spawn_points = world.get_map().get_spawn_points()
ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))

# bring pov to vehicle
spectator = world.get_spectator()
spectator.set_transform(ego_vehicle.get_transform())

ego_vehicle.set_autopilot(True)
time.sleep(60)