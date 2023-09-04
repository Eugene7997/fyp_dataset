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
# Create a transform to place the camera on top of the vehicle
camera_initial_transform = carla.Transform(carla.Location(z=1.5))
# We create the camera through a blueprint that defines its properties
camera_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
IM_WIDTH = 640*2
IM_HEIGHT = 480*2
camera_blueprint.set_attribute('image_size_x', str(IM_WIDTH))
camera_blueprint.set_attribute('image_size_y', str(IM_HEIGHT))
# We spawn the camera and attach it to our ego vehicle
camera = world.spawn_actor(camera_blueprint, camera_initial_transform, attach_to=ego_vehicle)
spectator.set_transform(ego_vehicle.get_transform())
camera.listen(lambda image: image.save_to_disk('./out/%06d.png' % image.frame))
time.sleep(5.0) # needed for camera images to be saved