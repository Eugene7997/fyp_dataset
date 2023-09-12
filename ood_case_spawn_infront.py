import numpy as np
from PIL import Image
import carla
import random
from agents.navigation.basic_agent import BasicAgent

def semanticSegmentationFlowCallback(image):
    image.convert(carla.ColorConverter.CityScapesPalette)
    buffer = np.frombuffer(image.raw_data, dtype=np.uint8)
    buffer = buffer.reshape(image.height, image.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
    Image.fromarray(buffer).save(f"./out/ss{image.frame}.png", "PNG")

def opticalFlowCallback(data):
    image = data.get_color_coded_flow()
    buffer = np.frombuffer(image.raw_data, dtype=np.uint8)
    buffer = buffer.reshape(image.height, image.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
    Image.fromarray(buffer).save(f"./out/of{data.frame}.jpeg", "JPEG")

# Carla world generation
client = carla.Client('localhost', 2000)
world = client.get_world()
client.load_world('Town05')

# spawn vehicle
vehicle_blueprints = world.get_blueprint_library().filter('vehicle.tesla.model3')
# spawn_points = world.get_map().get_spawn_points()
custom_defined_transform = carla.Transform(carla.Location(x=31.290208, y=-11.658614, z=1.980442), carla.Rotation(yaw=-90))
ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), custom_defined_transform)

# bring pov to vehicle
spectator = world.get_spectator()
spectator.set_transform(ego_vehicle.get_transform())

# set destination for vehicle
agent = BasicAgent(ego_vehicle)
destination = carla.Location(x=32.190208, y=-65.702446, z=1.980442)
agent.set_destination(destination)

# camera
camera_initial_transform = carla.Transform(carla.Location(z=2.5)) # Create a transform to place the camera on top of the vehicle
IM_WIDTH = 640*2
IM_HEIGHT = 480*2

# optical flow camera 
camera_optical_flow_blueprint = world.get_blueprint_library().find('sensor.camera.optical_flow')
camera_optical_flow_blueprint.set_attribute('image_size_x', str(IM_WIDTH))
camera_optical_flow_blueprint.set_attribute('image_size_y', str(IM_HEIGHT))
camera_optical_flow = world.spawn_actor(camera_optical_flow_blueprint, camera_initial_transform, attach_to=ego_vehicle)
spectator.set_transform(ego_vehicle.get_transform())
camera_optical_flow.listen(lambda data: opticalFlowCallback(data))

# semantic segmentation camera
camera_semantic_segmentation_blueprint = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
camera_semantic_segmentation_blueprint.set_attribute('image_size_x', str(IM_WIDTH))
camera_semantic_segmentation_blueprint.set_attribute('image_size_y', str(IM_HEIGHT))
camera_semantic_segmentation = world.spawn_actor(camera_semantic_segmentation_blueprint, camera_initial_transform, attach_to=ego_vehicle)
spectator.set_transform(ego_vehicle.get_transform())
# cc = carla.ColorConverter.CityScapesPalette
# camera_semantic_segmentation.listen(lambda image: image.save_to_disk('./out/%06d.png' % image.frame, cc))
camera_semantic_segmentation.listen(lambda data: semanticSegmentationFlowCallback(data))

# move the vehicle in a straight direction
counter = 0
custom_defined_transform_for_ood = carla.Transform(carla.Location(x=35.590208, y=-41.658614, z=1.980442), carla.Rotation(yaw=-90))
while True:
    if counter == 5000:
        world.spawn_actor(random.choice(vehicle_blueprints), custom_defined_transform_for_ood)
    if agent.done():
        print("The target has been reached, stopping the simulation")
        break
    counter = counter + 1
    ego_vehicle.apply_control(agent.run_step())