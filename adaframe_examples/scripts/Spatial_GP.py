#!/usr/bin/env python3
# license removed for brevity
# Adaptive sampling group of NTNU
# Tore Mo-Bjørkelund 2021
# contact: tore.mo-bjorkelund@ntnu.no

import numpy as np
import natural_cubic_spline


class SpatialGaussianProcess:
    def __init__(self,axes = 2, temporal = False):
        self.axes = axes
        self.temporal = temporal

        
