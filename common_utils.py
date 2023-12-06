import numpy as np
from PIL import Image
import carla

def save_optical_flow(image_input):
    buffer = np.frombuffer(image_input.raw_data, dtype=np.float32)
    buffer = buffer.reshape(image_input.height, image_input.width, 2) # 2d, horizontal and vertical
    np.save(f"./out/of{image_input.frame}.npy", buffer)

def semanticSegmentationFlowCallback(image, _sensor_queue, sensor_name):
    image.convert(carla.ColorConverter.CityScapesPalette)
    buffer = np.frombuffer(image.raw_data, dtype=np.uint8)
    buffer = buffer.reshape(image.height, image.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
    Image.fromarray(buffer).save(f"./out/ss{image.frame}.png", "PNG")
    _sensor_queue.put((image.frame, sensor_name))
    
def rgbCameraCallback(image, _sensor_queue, sensor_name):
    buffer = np.frombuffer(image.raw_data, dtype=np.uint8)
    buffer = buffer.reshape(image.height, image.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
    Image.fromarray(buffer).save(f"./out/rgb{image.frame}.png", "PNG")
    _sensor_queue.put((image.frame, sensor_name))

def opticalFlowCallback(data, _sensor_queue, sensor_name):
    image = data.get_color_coded_flow()
    buffer = np.frombuffer(image.raw_data, dtype=np.uint8)
    buffer = buffer.reshape(image.height, image.width, 4)[..., [2, 1, 0]]  # BGRA -> RGB
    Image.fromarray(buffer).save(f"./out/of{data.frame}.jpeg", "JPEG")
    _sensor_queue.put((data.frame, sensor_name))
    save_optical_flow(data)

IMAGE_WIDTH = 1280
IMAGE_HEIGHT = 960