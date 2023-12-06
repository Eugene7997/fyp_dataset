import numpy as np
from PIL import Image
import carla
import random
import time
from queue import Queue
from common_utils import *

# Carla world generation
client = carla.Client('localhost', 2000)
world = client.get_world()
client.load_world('Town10HD_Opt')
settings = world.get_settings()
settings.synchronous_mode = True # Enables synchronous mode
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

sensor_queue = Queue()
sensor_list = []

# spawn vehicle
vehicle_blueprints = world.get_blueprint_library().filter('vehicle.tesla.model3')
spawn_points = world.get_map().get_spawn_points()
ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))

# camera
camera_initial_transform = carla.Transform(carla.Location(x=1.1, z=1.2)) # Create a transform to place the camera on top of the vehicle

# rgb camera
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

# Call camera to capture images via callback functions
camera_rgb.listen(lambda data: rgbCameraCallback(data, sensor_queue, "rgb"))
camera_optical_flow.listen(lambda data: opticalFlowCallback(data, sensor_queue, "opticalFlow"))
camera_semantic_segmentation.listen(lambda data: semanticSegmentationFlowCallback(data, sensor_queue, "semanticSegmentation"))

ego_vehicle.set_autopilot(True)

# spawn other vehicles
no_of_other_vehicles = 5
for i in range(0,no_of_other_vehicles):
    temp = world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
    if temp == None:
        pass
    else:    
        temp.set_autopilot(True)

list_actor = world.get_actors()
for actor_ in list_actor:
    if isinstance(actor_, carla.TrafficLight):
        actor_.set_state(carla.TrafficLightState.Green) 
        actor_.set_green_time(1000.0)

# bring pov to vehicle
spectator = world.get_spectator()
spectator.set_transform(ego_vehicle.get_transform())

counter = 0
no_of_samples_desired = 10
while counter < no_of_samples_desired:
    world.tick()
    for _ in range(len(sensor_list)):
        while sensor_queue.empty():
            pass
        s_frame = sensor_queue.get(True, 1.0)
    counter += 1