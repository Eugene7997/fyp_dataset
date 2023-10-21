import numpy as np
from PIL import Image
import carla
import random
from agents.navigation.basic_agent import BasicAgent
from queue import Queue
from queue import Empty
import time

def write_to_csv(image_input, camera_type_name):    
    if camera_type_name == 'semanticSegmentation':
        # Assuming we are using original image. Original image has correct labels.
        buffer2 = np.frombuffer(image_input.raw_data, dtype=np.uint8)
        buffer2 = buffer2.reshape(image_input.height, image_input.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
        buffer2 = buffer2[:,:,0] # it seems that its the same values for all 3 R,G,B dimensions
        f=open('semanticSegmentation.csv','ab')
        np.savetxt(f,buffer2, delimiter=",", fmt="%i") # require fmt cuz values keeps converting to float
        f.close()
        return
    elif camera_type_name == 'opticalFlow':
        buffer2 = np.frombuffer(image_input.raw_data, dtype=np.float32)
        buffer2 = buffer2.reshape(image_input.height, image_input.width, 2) # 2d, horizontal and vertical
        buffer2 = buffer2.reshape(buffer2.shape[0], -1) # np save text does not allow 3d
        f=open('opticalFlow.csv','ab')
        np.savetxt(f,buffer2, delimiter=",")
        f.close()
        return
    elif camera_type_name == 'rgb':
        buffer2 = np.frombuffer(image_input.raw_data, dtype=np.uint8)
        buffer2 = buffer2.reshape(image_input.height, image_input.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
        buffer2 = buffer2.reshape(buffer2.shape[0], -1) # np save text does not allow 3d
        f=open('rgb.csv','ab')
        np.savetxt(f,buffer2, delimiter=",", fmt="%i") # require fmt cuz values keeps converting to float
        f.close()
        return

def semanticSegmentationFlowCallback(image, sensor_queue, sensor_name):
    image.convert(carla.ColorConverter.CityScapesPalette)
    buffer = np.frombuffer(image.raw_data, dtype=np.uint8)
    buffer = buffer.reshape(image.height, image.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
    Image.fromarray(buffer).save(f"./out/ss{image.frame}.png", "PNG")
    sensor_queue.put((image.frame, sensor_name))
    write_to_csv(image, sensor_name) 
    
def rgbCameraCallback(image, sensor_queue, sensor_name):
    buffer = np.frombuffer(image.raw_data, dtype=np.uint8)
    buffer = buffer.reshape(image.height, image.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
    Image.fromarray(buffer).save(f"./out/rgb{image.frame}.png", "PNG")
    sensor_queue.put((image.frame, sensor_name))
    write_to_csv(image, sensor_name) 

def opticalFlowCallback(data, sensor_queue, sensor_name):
    image = data.get_color_coded_flow()
    buffer = np.frombuffer(image.raw_data, dtype=np.uint8)
    buffer = buffer.reshape(image.height, image.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
    Image.fromarray(buffer).save(f"./out/of{data.frame}.jpeg", "JPEG")
    sensor_queue.put((data.frame, sensor_name))
    write_to_csv(data, sensor_name) 

# Carla world generation
client = carla.Client('localhost', 2000)
world = client.get_world()
client.load_world('Town10HD_Opt')
world.unload_map_layer(carla.MapLayer.Walls)

original_settings = world.get_settings()
settings = world.get_settings()
settings.fixed_delta_seconds = 0.05
settings.synchronous_mode = True

sensor_queue = Queue()
sensor_list = []

# spawn vehicle
vehicle_blueprints = world.get_blueprint_library().filter('vehicle.tesla.model3')
spawn_points = world.get_map().get_spawn_points()
ego_vehicle = world.spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
# camera
camera_initial_transform = carla.Transform(carla.Location(x=1.1, z=1.2))
IM_WIDTH = 640*2
IM_HEIGHT = 480*2

# semantic segmentation camera
camera_semantic_segmentation_blueprint = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
camera_semantic_segmentation_blueprint.set_attribute('image_size_x', str(IM_WIDTH))
camera_semantic_segmentation_blueprint.set_attribute('image_size_y', str(IM_HEIGHT))
camera_semantic_segmentation = world.spawn_actor(camera_semantic_segmentation_blueprint, camera_initial_transform, attach_to=ego_vehicle)
sensor_list.append(camera_semantic_segmentation)

# normal rgb camera
camera_rgb_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
camera_rgb_blueprint.set_attribute('image_size_x', str(IM_WIDTH))
camera_rgb_blueprint.set_attribute('image_size_y', str(IM_HEIGHT))
camera_rgb = world.spawn_actor(camera_rgb_blueprint, camera_initial_transform, attach_to=ego_vehicle)
sensor_list.append(camera_rgb)

# optical flow camera 
camera_optical_flow_blueprint = world.get_blueprint_library().find('sensor.camera.optical_flow')
camera_optical_flow_blueprint.set_attribute('image_size_x', str(IM_WIDTH))
camera_optical_flow_blueprint.set_attribute('image_size_y', str(IM_HEIGHT))
camera_optical_flow = world.spawn_actor(camera_optical_flow_blueprint, camera_initial_transform, attach_to=ego_vehicle)
sensor_list.append(camera_optical_flow)

# bring pov to vehicle
spectator = world.get_spectator()
spectator.set_transform(ego_vehicle.get_transform())

camera_rgb.listen(lambda data: rgbCameraCallback(data, sensor_queue, "rgb"))
camera_optical_flow.listen(lambda data: opticalFlowCallback(data, sensor_queue, "opticalFlow"))
camera_semantic_segmentation.listen(lambda data: semanticSegmentationFlowCallback(data, sensor_queue, "semanticSegmentation"))


ego_vehicle.set_autopilot(True)

world.apply_settings(settings)

counter = 0
while True:
    world.tick()
    for _ in range(len(sensor_list)):
        if not sensor_queue.empty():
            s_frame = sensor_queue.get(True, 1.0)
        else:
            time.sleep(1)
    if (counter == 10):
        ego_vehicle.set_transform(carla.Transform(carla.Location(x=-227.037689, y=162.543137, z=8.654385), carla.Rotation(pitch=1.956870, yaw=-1.042053, roll=0.000042)))
        ego_vehicle.set_autopilot(False)
        print("vehicle teleported")
    counter+=1
    if (counter == 50):
        break

