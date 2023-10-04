import numpy as np
from PIL import Image
import carla
import random
from agents.navigation.basic_agent import BasicAgent
import time

# Carla world generation
client = carla.Client('localhost', 2000)
world = client.get_world()
client.load_world('Town05')

original_settings = world.get_settings()
settings = world.get_settings()
settings.fixed_delta_seconds = 0.02
settings.synchronous_mode = True


# spawn vehicle
vehicle_blueprints = world.get_blueprint_library().filter('vehicle.tesla.model3')
# custom_defined_transform = carla.Transform(carla.Location(x=31.290208, y=-11.658614, z=1.980442), carla.Rotation(yaw=-90))
custom_defined_transform = carla.Transform(carla.Location(x=35.290208, y=-16.658614, z=0.980442), carla.Rotation(yaw=-90))
ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), custom_defined_transform)

# bring pov to vehicle
spectator = world.get_spectator()
spectator.set_transform(ego_vehicle.get_transform())

# set destination for vehicle
agent = BasicAgent(ego_vehicle)
destination = carla.Location(x=35.290208, y=-65.658614, z=0.980442)
agent.set_destination(destination)

world.apply_settings(settings)
# move the vehicle in a straight direction
while True:
    world.tick()
    if agent.done():
        print("The target has been reached, stopping the simulation")
        break
    ego_vehicle.apply_control(agent.run_step())