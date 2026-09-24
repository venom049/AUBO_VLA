#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np


class Camera:
    """
    Camera abstraction.

    Current:
    simulated image placeholder.

    Future:
    RealSense / GoPro / RGB-D camera.
    """

    def get_image(self):

        # fake RGB image
        image = np.zeros(
            (224,224,3),
            dtype=np.uint8
        )

        return image
