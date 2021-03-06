// Copyright (c) 2018, Smart Projects Holdings Ltd
// All rights reserved.
// See LICENSE file for license details.

#include <ugcs/vsm/coordinates.h>

float
ugcs::vsm::Normalize_angle_0_2pi(float a)
{
    float ret = fmod(a, 2 * M_PI);
    if (ret < 0) {
        return ret + 2 * M_PI;
    } else {
        return ret;
    }
}

float
ugcs::vsm::Normalize_angle_minuspi_pi(float a)
{
    return Normalize_angle_0_2pi(a + M_PI) - M_PI;
}
