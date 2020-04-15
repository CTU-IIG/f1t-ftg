/*
 *
 * FollowTheGap class (header)
 *
 * Authors: Anders Solberg Pedersen, Jaroslav Klapálek
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

#ifndef FOLLOW_THE_GAP_H_
#define FOLLOW_THE_GAP_H_

#include "obstacle.h"
#include "gap.h"
#include "corner.h"
#include "lidar_data.h"
#include <vector>
#include <cmath>
#include <iostream>
#include <algorithm>
#include <tuple>
#include <sstream>


namespace FollowTheGap {

    namespace AngleFilter {
        static size_t left_index;
        static size_t right_index;
    };

    // Parameteres
    static float kMaxRange = 0.9;
    static float g_fovAngleMax = M_PI/2+M_PI/16;
    static float g_goal_angle;
    // Constants
    static constexpr float kDistanceToCorner = 0.22;
    static constexpr int unsigned kPublishMessageBufferSize = 10;
    static constexpr int unsigned kSubscribeMessageBufferSize = 1;
    static constexpr float kTurnRadius = 0.3;
    static constexpr float kCarRadius = 0.4;
    static constexpr float kGapWeightCoefficient = 100; // 10
    static constexpr float kCornerWeightCoefficient = 100; // 10

    static constexpr float kFovAngle = M_PI/4;
    // Max range for obstacles to be taken into account in metres
    static constexpr float kTrackMinWidth = 0.35; // 1

    // Static objects
    static Obstacle obstacle_nhol_left(kTurnRadius, M_PI/2, kTurnRadius);
    static Obstacle obstacle_nhol_right(kTurnRadius, -M_PI/2, kTurnRadius);

    // Function declarations
    float FollowTheGapMethod(std::vector<Obstacle> obstacles, LidarData * lidar_data, std::vector<Obstacle> & gap_borders_out);
    float FtgWeightedAverageMethod(std::vector<Obstacle> obstacles, LidarData * lidar_data);
    float FollowTheCornerMethod(std::vector<Obstacle> obstacles, std::vector<Obstacle> & gap_borders_out);
    float FollowLargestVerticalGapMethod(std::vector<Obstacle> obstacles, std::vector<Obstacle> & gap_borders_out);
    float FtgWithAngleFilter(std::vector<Obstacle> obstacles, LidarData * lidar_data, std::vector<Obstacle> & gap_borders_out);
    std::vector<Gap> FindGapsAngle(std::vector<Obstacle> & obstacles);
    std::vector<Gap> FindGapsVerticalDistance(std::vector<Obstacle> & obstacles);
    float FindVerticalGapSafeAngle(Gap const & gap);
    float CalculateGapCenterAngle(Gap const &);
    float CalculateGapCenterAngleBasic(Gap const &);
    float CalculateFinalHeadingAngle(float const goal_angle, float const gap_center_angle, float const d_min, float const alpha);
    std::tuple<bool, float> Callback(const std::vector<Obstacle> & obstacles, LidarData * lidar_data, std::vector<Obstacle> & obstacles_out, std::vector<Obstacle> & gap_borders_out);

    void FilterObstacles(std::vector<Obstacle> & obstacles, const std::vector<Obstacle> & obstacles_in);
    Corner FindCorner(std::vector<Obstacle> & obstacles);
    float FindSafeCornerAngle(Corner & corner);

    // Exceptions
    class InvalidAngleException : public std::logic_error {
        public:
        InvalidAngleException(std::string const & msg)
            : std::logic_error(msg) {
        };
    };
    class CenterOutsideGapException : public std::logic_error {
        public:
        CenterOutsideGapException(std::string const & msg)
            : std::logic_error(msg) {
        };
    };
    class NoGapFoundException : public std::runtime_error {
        public:
        NoGapFoundException(std::string const & msg)
            : std::runtime_error(msg) {
        };
    };
};

#endif
