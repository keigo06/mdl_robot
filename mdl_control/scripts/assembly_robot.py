#!/usr/bin/env python3
""" Assembly Robot Class
"""

import numpy as np
import numpy.typing as npt

import logging as log

logger = log.getLogger(__name__)
log.basicConfig(
    level=log.DEBUG,
    format="%(message)s",
    datefmt="[%X]",
    handlers=[log.FileHandler(filename="logger.log")]
)


class Robot:
    def __init__(self, js_init: list[npt.NDArray] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]):
        self.name = "Robot"
        self.base_position = np.array([0.0, 0.0, 0.0])
        self.base_orientation = np.array([0.0, 0.0, 0.0, 1.0])
        self.m_size = 0.12

    def inverse_kinematics(self, pos: npt.NDArray, att: npt.NDArray):
        """
        Inverse Kinematics
        """
        pass
