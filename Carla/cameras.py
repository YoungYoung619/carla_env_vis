"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :


Author：Team Li
"""
import config
config.carla_module_init()
import carla
from carla import ColorConverter as cc
import weakref
from logger import logger
import numpy as np

from Carla.vehicles import BaseVehicle
from Carla.vehicles import TeslaModel3

from enum import Enum, unique
import pygame
import cv2
import time

from threading import Thread, Lock

@unique
class CameraType(Enum):
    RAW_RGB = 0
    RAW_DEPTH_CAMERA = 1
    GRAY_DEPTH_CAMERA = 2
    LOG_DEPTH_CAMERA = 3
    RAW_SEGMENTIC_CAMERA = 4
    CITY_SEGMENTIC_CAMERA = 5
    RAY_LIDAR = 6
    DYNAMIC_VISION_CAMERA = 7
    DISTORTED_RGB_CAMERA = 8

sensor_config = [['sensor.camera.rgb', cc.Raw, 'Camera RGB', {}],
                ['sensor.camera.depth', cc.Raw, 'Camera Depth (Raw)', {}],
                ['sensor.camera.depth', cc.Depth, 'Camera Depth (Gray Scale)', {}],
                ['sensor.camera.depth', cc.LogarithmicDepth, 'Camera Depth (Logarithmic Gray Scale)', {}],
                ['sensor.camera.semantic_segmentation', cc.Raw, 'Camera Semantic Segmentation (Raw)', {}],
                ['sensor.camera.semantic_segmentation', cc.CityScapesPalette,
                 'Camera Semantic Segmentation (CityScapes Palette)', {}],
                ['sensor.lidar.ray_cast', None, 'Lidar (Ray-Cast)', {'range': '50'}],
                ['sensor.camera.dvs', cc.Raw, 'Dynamic Vision Sensor', {}],
                ['sensor.camera.rgb', cc.Raw, 'Camera RGB Distorted',
                 {'lens_circle_multiplier': '3.0',
                  'lens_circle_falloff': '3.0',
                  'chromatic_aberration_intensity': '0.5',
                  'chromatic_aberration_offset': '0'}]]

class CameraDecarator(BaseVehicle):
    """camera装饰器，改变BaseVehicle的state()行为，让其返回camera形式的state"""
    def __init__(self, vehicle: BaseVehicle, gamma_correction=1.5, height=1280, width=720):
        super().__init__()
        self.vehicle = vehicle
        self.camera_manager = CameraManager(vehicle.player, gamma_correction, height, width)
        self.set_camera()

    def next_camera(self):
        camera_type_value = (self.current_camera.value + 1) % (len(CameraType))
        self.set_camera(CameraType(camera_type_value))

    def set_camera(self, camera_type:CameraType=CameraType.DISTORTED_RGB_CAMERA):
        self.current_camera = camera_type
        self.camera_manager.set_sensor(index=camera_type.value)

    def state(self):
        # self.camera_manager.wait_for_data_ready()
        return self.camera_manager.image

class CameraManager(object):
    def __init__(self, parent_actor, gamma_correction:float=1.5, height:int=1280, width:int=720):
        self.height = height
        self.width = width
        self.sensor = None
        self.surface = None
        self.image = None
        self._parent = parent_actor
        self.recording = False
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        Attachment = carla.AttachmentType
        self._camera_transforms = [
            (carla.Transform(carla.Location(x=-5.5, z=2.5), carla.Rotation(pitch=8.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=1.6, z=1.7)), Attachment.Rigid),
            (carla.Transform(carla.Location(x=5.5, y=1.5, z=1.5)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-8.0, z=6.0), carla.Rotation(pitch=6.0)), Attachment.SpringArm),
            (carla.Transform(carla.Location(x=-1, y=-bound_y, z=0.5)), Attachment.Rigid)]
        self.transform_index = 0
        self.sensors = sensor_config
        world = self._parent.get_world()
        bp_library = world.get_blueprint_library()
        for item in self.sensors:
            bp = bp_library.find(item[0])
            if item[0].startswith('sensor.camera'):
                bp.set_attribute('image_size_x', str(width))
                bp.set_attribute('image_size_y', str(height))
                bp.set_attribute('sensor_tick', str(0.01))
                if bp.has_attribute('gamma'):
                    bp.set_attribute('gamma', str(gamma_correction))
                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
            elif item[0].startswith('sensor.lidar'):
                self.lidar_range = 50

                for attr_name, attr_value in item[3].items():
                    bp.set_attribute(attr_name, attr_value)
                    if attr_name == 'range':
                        self.lidar_range = float(attr_value)

            item.append(bp)
        self.index = None
        self.data_ready = False
        self.lock = Lock()

    def toggle_camera(self):
        self.transform_index = (self.transform_index + 1) % len(self._camera_transforms)
        self.set_sensor(self.index, force_respawn=True)

    def set_sensor(self, index, force_respawn=False):
        self.data_ready = False
        index = index % len(self.sensors)
        logger.info("sensor: %s"%(str(self.sensors[index])))
        needs_respawn = True if self.index is None else \
            (force_respawn or (self.sensors[index][2] != self.sensors[self.index][2]))
        # print(needs_respawn)
        if needs_respawn:
            if self.sensor is not None:
                self.sensor.destroy()
                self.surface = None
                self.image = None
            self.sensor = self._parent.get_world().spawn_actor(
                self.sensors[index][-1],
                self._camera_transforms[self.transform_index][0],
                attach_to=self._parent,
                attachment_type=self._camera_transforms[self.transform_index][1])
            # We need to pass the lambda a weak reference to self to avoid
            # circular reference.
            # weak_self = weakref.ref(self)
            self.sensor.listen(lambda image: self._parse_image(image))

        self.index = index

    def next_sensor(self):
        self.set_sensor(self.index + 1)

    def toggle_recording(self):
        self.recording = not self.recording
        logger.info('Recording %s' % ('On' if self.recording else 'Off'))

    def render(self, display):
        # if self.surface is not None:
        #     display.blit(self.surface, (0, 0))

        # self.lock.acquire()
        if self.image is not None:
            a = cv2.resize(cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB),
                           (int(256*1.5), int(144*1.5)))
            print('test2:', a[108][192])
            cv2.imshow('test2', a)
            cv2.waitKey(1)
        # self.lock.release()

    def is_ready(self):
        return self.data_ready

    def wait_for_data_ready(self):
        while not self.is_ready():
            pass

    def _parse_image(self, image):
        t0 = time.time()
        # self = weak_self()
        # if not self:
        #     return
        # self.lock.acquire()
        if self.sensors[self.index][0].startswith('sensor.lidar'):
            points = np.frombuffer(image.raw_data, dtype=np.dtype('f4'))
            points = np.reshape(points, (int(points.shape[0] / 4), 4))
            lidar_data = np.array(points[:, :2])
            lidar_data *= min((self.height, self.width)) / (2.0 * self.lidar_range)
            lidar_data += (0.5 * self.height, 0.5 * self.width)
            lidar_data = np.fabs(lidar_data)  # pylint: disable=E1111
            lidar_data = lidar_data.astype(np.int32)
            lidar_data = np.reshape(lidar_data, (-1, 2))
            lidar_img_size = (self.height, self.width, 3)
            lidar_img = np.zeros((lidar_img_size), dtype=np.uint8)
            lidar_img[tuple(lidar_data.T)] = (255, 255, 255)
            # self.surface = pygame.surfarray.make_surface(lidar_img)
            res = lidar_img
        elif self.sensors[self.index][0].startswith('sensor.camera.dvs'):
            # Example of converting the raw_data from a carla.DVSEventArray
            # sensor into a NumPy array and using it as an image
            dvs_events = np.frombuffer(image.raw_data, dtype=np.dtype([
                ('x', np.uint16), ('y', np.uint16), ('t', np.int64), ('pol', np.bool)]))
            dvs_img = np.zeros((image.height, image.width, 3), dtype=np.uint8)
            # Blue is positive, red is negative
            dvs_img[dvs_events[:]['y'], dvs_events[:]['x'], dvs_events[:]['pol'] * 2] = 255
            res = dvs_img
            # self.surface = pygame.surfarray.make_surface(dvs_img.swapaxes(0, 1))
        else:
            image.convert(self.sensors[self.index][1])
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]
            array = array[:, :, ::-1]
            # self.surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
            res = array
        self.surface = pygame.surfarray.make_surface(res.swapaxes(0, 1))
        self.image = res
        # self.lock.release()

        # self.render(None)
        # print("FPS: ",int(1./(time.time()-t0)))
        # if self.image is not None:
        #     a = cv2.resize(cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB),
        #                    (int(256*1.5), int(144*1.5)))
        #     print('test1:', a[108][192])
        #     cv2.imshow('test1', a)
        #     cv2.waitKey(1)
        self.data_ready = True

if __name__ == '__main__':


    tesla = TeslaModel3(role_name='role1')
    tesla = CameraDecarator(tesla)
