import numpy as np
from PIL import Image
import carla
import random
from agents.navigation.basic_agent import BasicAgent
import time
from queue import Queue
from common_utils import *

# Carla world generation
client = carla.Client('localhost', 2000)
world = client.get_world()
client.load_world('Town05')
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05

sensor_queue = Queue()
sensor_list = []

vehicle_blueprints = world.get_blueprint_library().filter('vehicle.tesla.model3')

# spawn floating vehicle
custom_defined_transform_for_ood = carla.Transform(carla.Location(x=32.290208, y=-21.658614, z=0.2), carla.Rotation(yaw=-90))
floating_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), custom_defined_transform_for_ood)
# set destination for vehicles
agent = BasicAgent(floating_vehicle)
destination = carla.Location(x=33, y=-70, z=1.980442)
agent.set_destination(destination)

world.apply_settings(settings)

# spawn ego vehicle
custom_defined_transform = carla.Transform(carla.Location(x=31.290208, y=-11.658614, z=0.2), carla.Rotation(yaw=-90))
ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), custom_defined_transform)

# camera
camera_initial_transform = carla.Transform(carla.Location(x=1.1, z=1.2))

#rgb camera
camera_rgb_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
camera_rgb_blueprint.set_attribute('image_size_x', str(IMAGE_WIDTH))
camera_rgb_blueprint.set_attribute('image_size_y', str(IMAGE_HEIGHT))
camera_rgb = world.spawn_actor(camera_rgb_blueprint, camera_initial_transform, attach_to=ego_vehicle)
sensor_list.append(camera_rgb)

# optical flow camera 
camera_optical_flow_blueprint = world.get_blueprint_library().find('sensor.camera.optical_flow')
camera_optical_flow_blueprint.set_attribute('image_size_x', str(IMAGE_WIDTH))
camera_optical_flow_blueprint.set_attribute('image_size_y', str(IMAGE_HEIGHT))
camera_optical_flow = world.spawn_actor(camera_optical_flow_blueprint, camera_initial_transform, attach_to=ego_vehicle)
sensor_list.append(camera_optical_flow)

# semantic segmentation camera
camera_semantic_segmentation_blueprint = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
camera_semantic_segmentation_blueprint.set_attribute('image_size_x', str(IMAGE_WIDTH))
camera_semantic_segmentation_blueprint.set_attribute('image_size_y', str(IMAGE_HEIGHT))
camera_semantic_segmentation = world.spawn_actor(camera_semantic_segmentation_blueprint, camera_initial_transform, attach_to=ego_vehicle)
sensor_list.append(camera_semantic_segmentation)

camera_rgb.listen(lambda data: rgbCameraCallback(data, sensor_queue, "rgb"))
camera_optical_flow.listen(lambda data: opticalFlowCallback(data, sensor_queue, "opticalFlow"))
camera_semantic_segmentation.listen(lambda data: semanticSegmentationFlowCallback(data, sensor_queue, "semanticSegmentation"))

# bring pov to vehicle
spectator = world.get_spectator()
spectator.set_transform(ego_vehicle.get_transform())

ego_vehicle.set_autopilot(True)

# move the vehicle in a straight direction
while True:
    floating_vehicle.apply_control(agent.run_step())
    world.tick()
    if agent.done():
        print("The target has been reached, stopping the simulation")
        break
    if floating_vehicle.get_transform().location.y < -25:
        floating_vehicle.set_enable_gravity(False)
    for _ in range(len(sensor_list)):
        while sensor_queue.empty():
            pass
        s_frame = sensor_queue.get(True, 1.0)
