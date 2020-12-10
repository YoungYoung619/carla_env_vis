"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :


Author：Team Li
"""
import config
config.carla_module_init()
import carla
import abc
import random

__all__ = ['']

class BaseVehicle():
    def __init__(self, role_name:str, filter:str):
        """初始化"""
        self.player = None
        self.player_max_speed = 1.589
        self.player_max_speed_fast = 3.713
        self._actor_filter = filter
        self.role_name = role_name
        pass

    def spwan_at(self, world, spawn_point):
        blueprint = random.choice(world.get_blueprint_library().filter(self._actor_filter))
        blueprint.set_attribute('role_name', self.role_name)
        if blueprint.has_attribute('color'):
            color = random.choice(blueprint.get_attribute('color').recommended_values)
            blueprint.set_attribute('color', color)
        if blueprint.has_attribute('driver_id'):
            driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
            blueprint.set_attribute('driver_id', driver_id)
        if blueprint.has_attribute('is_invincible'):
            blueprint.set_attribute('is_invincible', 'true')
        # set the max speed
        if blueprint.has_attribute('speed'):
            self.player_max_speed = float(blueprint.get_attribute('speed').recommended_values[1])
            self.player_max_speed_fast = float(blueprint.get_attribute('speed').recommended_values[2])

        if self.player is None:
            self.player = world.try_spawn_actor(blueprint, spawn_point)
        else:
            self.player.stop()
            self.player.destroy()
            self.player = world.try_spawn_actor(blueprint, spawn_point)
        return self.player


    @abc.abstractmethod
    def state(self):
        """返回汽车状态，可以是动力学信息，也可以是传感器信息"""
        pass

class TeslaModel3(BaseVehicle):
    def __init__(self, role_name:str):
        BaseVehicle.__init__(self, role_name, "vehicle.tesla.model3")

    def state(self):
        """默认返回位姿和位姿变化"""
        return self.player.get_transform(), self.player.get_velocity(), self.player.get_angular_velocity()

