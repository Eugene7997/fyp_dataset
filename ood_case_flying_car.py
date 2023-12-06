import carla
import random
from agents.navigation.basic_agent import BasicAgent
from queue import Queue
import time
from common_utils import *

# Carla world generation
client = carla.Client('localhost', 2000)
world = client.get_world()
client.load_world('Town05')

original_settings = world.get_settings()
settings = world.get_settings()
settings.fixed_delta_seconds = 0.05
settings.synchronous_mode = True

# spawn ego vehicle
vehicle_blueprints = world.get_blueprint_library().filter('vehicle.tesla.model3')
custom_defined_transform = carla.Transform(carla.Location(x=31.290208, y=-11.658614, z=0.02), carla.Rotation(yaw=-90))
ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), custom_defined_transform)

# set destination for vehicle
agent = BasicAgent(ego_vehicle)
destination = carla.Location(x=32.190208, y=-65.702446, z=1.980442)
agent.set_destination(destination)

world.apply_settings(settings) # must be here after the destination is set

sensor_queue = Queue()
sensor_list = []

# camera
camera_initial_transform = carla.Transform(carla.Location(x=1.1, z=1.2))

# semantic segmentation camera
camera_semantic_segmentation_blueprint = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
camera_semantic_segmentation_blueprint.set_attribute('image_size_x', str(IMAGE_WIDTH))
camera_semantic_segmentation_blueprint.set_attribute('image_size_y', str(IMAGE_HEIGHT))
camera_semantic_segmentation = world.spawn_actor(camera_semantic_segmentation_blueprint, camera_initial_transform, attach_to=ego_vehicle)
sensor_list.append(camera_semantic_segmentation)

# normal rgb camera
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

# bring pov to vehicle
spectator = world.get_spectator()
spectator.set_transform(ego_vehicle.get_transform())

camera_rgb.listen(lambda data: rgbCameraCallback(data, sensor_queue, "rgb"))
camera_optical_flow.listen(lambda data: opticalFlowCallback(data, sensor_queue, "opticalFlow"))
camera_semantic_segmentation.listen(lambda data: semanticSegmentationFlowCallback(data, sensor_queue, "semanticSegmentation"))

# move the vehicle in a straight direction
counter = 0
custom_defined_transform_for_flying_vehicle = carla.Transform(carla.Location(x=35.590208, y=-41.658614, z=0.02), carla.Rotation(yaw=-90))

# you need to adjust this. The more image captures, the lower this value needs to be for OOD car to spawn in time.
value_for_spawning_flying_vehicle = 50
counter_for_transform = 0
while True:
    ego_vehicle.apply_control(agent.run_step())
    world.tick()
    for _ in range(len(sensor_list)):
        while sensor_queue.empty():
            pass
        s_frame = sensor_queue.get(True, 1.0)
    if counter == value_for_spawning_flying_vehicle:
        flying_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), custom_defined_transform_for_flying_vehicle)
        print("vehicle spawned")
    if counter >= value_for_spawning_flying_vehicle: 
        flying_vehicle.set_transform(carla.Transform(carla.Location(x=35.590208, y=-30.658614-counter_for_transform, z=0.2+(counter_for_transform/10)), carla.Rotation(yaw=-90.0)))
        # print(flying_vehicle.get_transform())
        counter_for_transform += 1
    if agent.done():
        print("The target has been reached, stopping the simulation")
        break
    counter = counter + 1