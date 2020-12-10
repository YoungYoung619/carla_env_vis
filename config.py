"""
Copyright (c) College of Mechatronics and Control Engineering, Shenzhen University.
All rights reserved.

Description :


Author：Team Li
"""
import sys
egg_file = "E:\game\CARLA_0.9.10\PythonAPI\carla\dist\carla-0.9.10-py3.7-win-amd64.egg"

def carla_module_init():
    sys.path.append(egg_file)
