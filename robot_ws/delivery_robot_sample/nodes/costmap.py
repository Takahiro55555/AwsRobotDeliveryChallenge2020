#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np

from nav_msgs.msg import OccupancyGrid

class Costmap():
    def __init__(self, array, height, width, resolution, origin):
        self.__array = array
        self.__height = height
        self.__width = width
        self.__resolution = resolution  # The map resolution [m/cell]
        self.__origin = origin

    def get_array(self):
        return self.__array
    
    # TODO: 適切な関数名をつける
    def convert_array(self, func):
        h, w = self.__height, self.__width
        return np.array(map(func, self.__array.reshape(h*w)), dtype="uint8").reshape(h, w)

    def get_size(self):
        return self.__height, self.__width

    def get_resolution(self):
        return self.__resolution

    def get_origin(self):
        return self.__origin

    def converte_occupancy_grid(self):
        grid = OccupancyGrid()
        grid.data = tuple(self.__array.reshape(self.__height * self.__width))
        grid.info.resolution = self.__resolution
        grid.info.origin = self.__origin
        grid.info.height = self.__height
        grid.info.width = self.__width
        return grid
