import carla
import random
import numpy as np
from PIL import Image
import cv2
import torch
from models.experimental import attempt_load
from utils.general import non_max_suppression

def process_image(image):
    # 将图像数据转换为numpy数组
    image_data = np.array(image.raw_data)
    # 从数组中提取RGB通道
    rgb_image = image_data.reshape((image.height, image.width, 4))[:, :, :3]
    # 将图像转换为PIL图像对象
    pil_image = Image.fromarray(rgb_image)
    return pil_image

def add_camera_blueprint(blueprint_library):
    camera_bp = blueprint_library.find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '1920')
    camera_bp.set_attribute('image_size_y', '1080')
    camera_bp.set_attribute('fov', '110')
    return camera_bp

def add_camera(world, blueprint_library, transform):
    camera_bp = add_camera_blueprint(blueprint_library)
    camera_transform = transform
    camera_transform.location.z += 1.5
    camera = world.spawn_actor(camera_bp, camera_transform)
    return camera

def pixel_to_world(u, v, intrinsic, extrinsic):
    # 将像素坐标系转换为相机坐标系
    x = (u - intrinsic[0][2]) / intrinsic[0][0]
    y = (v - intrinsic[1][2]) / intrinsic[1][1]
    z = 1

    # 将相机坐标系转换为世界坐标系
    pixel_pos = np.array([x, y, z])
    world_pos = np.dot(extrinsic.get_matrix(), pixel_pos)
    return world_pos

def camera_callback(model, camera):

    intrinsic = camera.intrinsic
    extrinsic = camera.get_transform()
    def _camera_callback(image):
        # 将图像转换为numpy数组
        image_array = np.frombuffer(image.raw_data, dtype=np.uint8)
        image_array = np.reshape(image_array, (image.height, image.width, 4))

        # 将BGRA格式转换为BGR格式
        img = image_array[:, :, :3]

        # 进行目标检测
        results = model(img)

        # 非极大值抑制
        results = non_max_suppression(results)

        # 获取车辆类别的目标位置
        for result in results:
            for detection in result:
                if detection[5] == 2:  # 车辆类别的标签为2
                    x1, y1, x2, y2 = detection[:4]
                    world_pos = pixel_to_world((x1 + x2) / 2, (y1 + y2) / 2, intrinsic, extrinsic)
                    print(world_pos)



    return _camera_callback

def main():
    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        # 获取摄像头蓝图
        camera_bp = add_camera_blueprint(blueprint_library)

        # 加载模型
        model = attempt_load('yolov5s.pt', map_location=torch.device('cpu'))

        # 生成10个车辆和10个相机
        for i in range(10):
            camera_transform = random.choice(world.get_map().get_spawn_points())
            camera_transform.location.z += 1.5
            camera_transform.rotation.pitch -= 15.0
            camera = world.spawn_actor(camera_bp, camera_transform)
            camera.listen(camera_callback(model, camera))

        # 等待仿真环境运行
        world.wait_for_tick()

    finally:
        for actor in world.get_actors():
            actor.destroy()

if __name__ == '__main__':
    main()