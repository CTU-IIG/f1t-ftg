/*
 *
 * Gap class (header)
 *
 * Authors: Anders Solberg Pedersen
 * Copyright (C) 2020 Czech Technical University in Prague
 *
 * This file is a part of follow_the_gap.
 *
 * follow_the_gap is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * follow_the_gap is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with follow_the_gap. If not, see <https://www.gnu.org/licenses/>.
 *
 */

#ifndef _GAP_H_
#define _GAP_H_

#include "obstacle.h"
#include <iostream>

namespace FollowTheGap {
    class Gap {
        public:
        Gap(Obstacle const & o1, Obstacle const & o2);
        float angle_left;
        float angle_right;
        float gap_size;
        // gap_distance is the distance between the obstacle centres
        float gap_distance;
        Obstacle const * obstacle_left;
        Obstacle const * obstacle_right;
    };
    std::ostream & operator<<(std::ostream & os, Gap const & g);
};


#endif // _GAP_H_
