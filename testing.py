import carla
import random
import time

# Connect to the client and retrieve the world object
client = carla.Client('localhost', 2000)
world = client.get_world()
client.load_world('Town05')

# Retrieve the spectator object
spectator = world.get_spectator()
# Get the location and rotation of the spectator through its transform
spectator_transform = spectator.get_transform()
spectator_location = spectator_transform.location
spectator_rotation = spectator_transform.rotation
# Set the spectator with an empty transform
spectator.set_transform(carla.Transform())
# This will set the spectator at the origin of the map, with 0 degrees
# pitch, yaw and roll - a good way to orient yourself in the map

vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
# Get the map's spawn points
spawn_points = world.get_map().get_spawn_points()
for i in range(0,50):
    world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))

ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))

# camera_initial_transform = carla.Transform(carla.Location(z=1.5)) # Create a transform to place the camera on top of the vehicle
# camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb') # We create the camera through a blueprint that defines its properties
# IM_WIDTH = 640*2
# IM_HEIGHT = 480*2
# camera_blueprint.set_attribute('image_size_x', str(IM_WIDTH))
# camera_blueprint.set_attribute('image_size_y', str(IM_HEIGHT))
# camera = world.spawn_actor(camera_blueprint, camera_initial_transform, attach_to=ego_vehicle) # We spawn the camera and attach it to our ego vehicle
# spectator.set_transform(ego_vehicle.get_transform())
# camera.listen(lambda image: image.save_to_disk('./out/%06d.png' % image.frame))
# time.sleep(5.0) # needed for camera images to be saved


# semantic segmentation camera
# In https://carla.readthedocs.io/en/0.9.14/python_api/#carla.Sensor.listen, check for code snippet
# https://carla.readthedocs.io/en/0.9.14/ref_sensors/#semantic-segmentation-camera
camera_initial_transform = carla.Transform(carla.Location(z=2.5)) # Create a transform to place the camera on top of the vehicle
camera_semantic_segmentation_blueprint = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
IM_WIDTH = 640*2
IM_HEIGHT = 480*2
camera_semantic_segmentation_blueprint.set_attribute('image_size_x', str(IM_WIDTH))
camera_semantic_segmentation_blueprint.set_attribute('image_size_y', str(IM_HEIGHT))
camera_semantic_segmentation = world.spawn_actor(camera_semantic_segmentation_blueprint, camera_initial_transform, attach_to=ego_vehicle)
spectator.set_transform(ego_vehicle.get_transform())
cc = carla.ColorConverter.CityScapesPalette
camera_semantic_segmentation.listen(lambda image: image.save_to_disk('./out/%06d.png' % image.frame, cc))
# ego_vehicle.set_autopilot(True)
# # time.sleep(5.0) # needed for camera images to be saved


import numpy as np
from PIL import Image

# References: https://github.com/carla-simulator/carla/issues/4610 , https://carla.readthedocs.io/en/0.9.14/python_api/#carlaopticalflowimage, https://carla.readthedocs.io/en/latest/ref_sensors/#optical-flow-camera
def opticalFlowCallback(data):
    image = data.get_color_coded_flow()
    # print(data)
    # print(type(data))
    # print(image)
    # print(type(image))
    buffer = np.frombuffer(image.raw_data, dtype=np.uint8)
    buffer = buffer.reshape(image.height, image.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
    Image.fromarray(buffer).save(f"./out/{data.frame}.jpeg", "JPEG")
    # Image.fromarray(buffer).save("./out/asd.jpeg", "JPEG")

def opticalFlowCallback2(data):
    image = data.get_color_coded_flow()
    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
    img[:,:,3] = 255
    Image.fromarray(img).save("./out/asd.jpeg") # does not work

camera_initial_transform = carla.Transform(carla.Location(z=2.5)) # Create a transform to place the camera on top of the vehicle
camera_blueprint_optical_flow_blueprint = world.get_blueprint_library().find('sensor.camera.optical_flow')
IM_WIDTH = 640*2
IM_HEIGHT = 480*2
camera_blueprint_optical_flow_blueprint.set_attribute('image_size_x', str(IM_WIDTH))
camera_blueprint_optical_flow_blueprint.set_attribute('image_size_y', str(IM_HEIGHT))
camera_optical_flow = world.spawn_actor(camera_blueprint_optical_flow_blueprint, camera_initial_transform, attach_to=ego_vehicle)
spectator.set_transform(ego_vehicle.get_transform())
camera_optical_flow.listen(lambda data: opticalFlowCallback(data))
ego_vehicle.set_autopilot(True)
time.sleep(5.0) # needed for camera images to be saved
