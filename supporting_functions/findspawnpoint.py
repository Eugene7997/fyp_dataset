import carla

client = carla.Client('localhost', 2000)
world = client.get_world()
client.load_world('Town05')

current_map = world.get_map()
spawn_points = current_map.get_spawn_points()
for point_index in range(len(spawn_points)):
    print(point_index, spawn_points[point_index])