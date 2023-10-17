import numpy as np
from PIL import Image
import carla
import random
import time
from queue import Queue
import csv
import pandas as pd
import base64

def write_to_csv(image_input):
    # with open('data.csv', 'w', newline='') as csvfile:
    #     imageToMatrice = np.asarray(image_input)
    #     flattened_array = imageToMatrice.flatten()
    #     a = ','.join(map(str, flattened_array))
    #     csv_writer = csv.writer(csvfile)
    #     csv_writer.writerow(a)

    # with open('data.csv', 'w', newline='') as csvfile:
        # imageToMatrice = Image.fromarray(image_input.astype(np.uint8),'RGB')
        # a = ','.join(map(str, imageToMatrice))
        # csv_writer = csv.writer(csvfile)
        # csv_writer.writerow(image_input)
    
    # TODO:
    # how to separate each image in csv? Newline? row size?
    # implement for other cameras

    # Assuming we are using original image. Original image has correct labels.
    buffer2 = np.frombuffer(image_input.raw_data, dtype=np.uint8)
    buffer2 = buffer2.reshape(image_input.height, image_input.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
    buffer2 = buffer2[:,:,0] # it seems that its the same values for all 3 R,G,B dimensions
    print(f"buffer2: {buffer2}")
    print(f"buffer2: {type(buffer2)}")
    print(f"buffer2: {buffer2.shape}")
    f=open('data.csv','ab')
    np.savetxt(f,buffer2, delimiter=",", fmt="%i") # require fmt cuz values keeps converting to float
    f.close()

    # np.save("data.csv", image_input)

def semanticSegmentationFlowCallback(image, sensor_queue, sensor_name):
    write_to_csv(image) 
    image.convert(carla.ColorConverter.CityScapesPalette)
    buffer = np.frombuffer(image.raw_data, dtype=np.uint8)
    buffer = buffer.reshape(image.height, image.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
    Image.fromarray(buffer).save(f"./out/ss{image.frame}.png", "PNG")
    sensor_queue.put((image.frame, sensor_name))

    # write_to_csv(buffer)

    # img_data = buffer.tobytes()
    # encoded_img_data = base64.b64encode(buffer).decode('utf-8')
    # global df
    # df = df.append(pd.Series([encoded_img_data]), ignore_index=True)
    
def rgbCameraCallback(image, sensor_queue, sensor_name):
    buffer = np.frombuffer(image.raw_data, dtype=np.uint8)
    buffer = buffer.reshape(image.height, image.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
    Image.fromarray(buffer).save(f"./out/rgb{image.frame}.png", "PNG")
    sensor_queue.put((image.frame, sensor_name))

def opticalFlowCallback(data, sensor_queue, sensor_name):
    image = data.get_color_coded_flow()
    buffer = np.frombuffer(image.raw_data, dtype=np.uint8)
    buffer = buffer.reshape(image.height, image.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
    Image.fromarray(buffer).save(f"./out/of{data.frame}.jpeg", "JPEG")
    sensor_queue.put((data.frame, sensor_name))

# Carla world generation
client = carla.Client('localhost', 2000)
world = client.get_world()
client.load_world('Town05')
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
camera_initial_transform = carla.Transform(carla.Location(z=2.5)) # Create a transform to place the camera on top of the vehicle
IM_WIDTH = 640*2
IM_HEIGHT = 480*2

#rgb camera
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

# semantic segmentation camera
camera_semantic_segmentation_blueprint = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
camera_semantic_segmentation_blueprint.set_attribute('image_size_x', str(IM_WIDTH))
camera_semantic_segmentation_blueprint.set_attribute('image_size_y', str(IM_HEIGHT))
camera_semantic_segmentation = world.spawn_actor(camera_semantic_segmentation_blueprint, camera_initial_transform, attach_to=ego_vehicle)
sensor_list.append(camera_semantic_segmentation)

camera_rgb.listen(lambda data: rgbCameraCallback(data, sensor_queue, "rgb"))
camera_optical_flow.listen(lambda data: opticalFlowCallback(data, sensor_queue, "opticalFlow"))
camera_semantic_segmentation.listen(lambda data: semanticSegmentationFlowCallback(data, sensor_queue, "semanticSegmentation"))

ego_vehicle.set_autopilot(True)

# spawn other vehicles
for i in range(0,5):
    temp = world.try_spawn_actor(random.choice(vehicle_blueprints), random.choice(spawn_points))
    if temp == None:
        pass
    else:    
        temp.set_autopilot(True)

list_actor = world.get_actors()
for actor_ in list_actor:
    if isinstance(actor_, carla.TrafficLight):
        # for any light, first set the light state, then set time. for yellow it is 
        # carla.TrafficLightState.Yellow and Red it is carla.TrafficLightState.Red
        actor_.set_state(carla.TrafficLightState.Green) 
        actor_.set_green_time(1000.0)
        # actor_.set_green_time(5000.0)
        # actor_.set_yellow_time(1000.0)

# bring pov to vehicle
spectator = world.get_spectator()
spectator.set_transform(ego_vehicle.get_transform())

time.sleep(10)

counter = 0
while True:
    world.tick()
    for _ in range(len(sensor_list)):
        s_frame = sensor_queue.get(True, 1.0)
    if counter == 30:
        break
    counter += 1