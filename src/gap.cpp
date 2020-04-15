/*
 *
 * Gap class
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

#include "gap.h"
#include <cmath>

namespace FollowTheGap {
    Gap::Gap(Obstacle const & o1, Obstacle const & o2)
        : angle_left(o1.angle_right), angle_right(o2.angle_left), gap_size(std::abs(angle_left-angle_right)),
        gap_distance(o1.DistanceBetweenObstacleCentres(o2)),
        obstacle_left(&o1), obstacle_right(&o2)
    {
    };
    std::ostream & operator<<(std::ostream & os, Gap const & g) {
        os << "angle_left: " << g.angle_left << " ";
        os << "angle_right: " << g.angle_right << " ";
        os << "gap size: " << g.gap_size;
        return os;
    };
}

