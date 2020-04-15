/*
 *
 * FollowTheGap class
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

#include "follow_the_gap.h"

float last_final_heading_angle = 0;

using namespace FollowTheGap;

static Obstacle const & FindNearestObstacle(std::vector<Obstacle> const & obstacles) {
    return *std::min_element(obstacles.cbegin(), obstacles.cend(), [](Obstacle const & a, Obstacle const & b) {
        return a.distance < b.distance;
    });
}

float FollowTheGap::FollowTheGapMethod(std::vector<Obstacle> obstacles, LidarData * lidar_data, std::vector<Obstacle> & gap_borders_out) {
    std::vector<Gap>::const_iterator largest_gap;
    std::vector<Gap> gaps;
    float final_heading_angle;
    try {
        gaps = FindGapsAngle(obstacles);
        largest_gap = std::max_element(gaps.cbegin(), gaps.cend(),[](Gap const & a, Gap const & b){
            return a.gap_size < b.gap_size;
        });
    } catch (InvalidAngleException const & e) {
        std::cerr << "ERROR: Invalid angle encountered when creating gap array: " << std::endl;
        std::cerr << e.what() << std::endl;
        throw NoGapFoundException("Found invalid angle.");
    }

    if ( largest_gap == gaps.cend() ) {
        throw NoGapFoundException("No gap found");
    }
    else {
        float gap_center_angle;
        try {
            gap_center_angle = CalculateGapCenterAngle(*largest_gap);
        } catch (InvalidAngleException const & e) {
            std::cerr << "ERROR: Exception occurred when calculating gap centre angle" << std::endl;
            std::cerr << e.what() << std::endl;
            throw NoGapFoundException("Found invalid gap center angle");
        } catch (const CenterOutsideGapException&) {
            // Fall back to the CalculateGapCenterAngleBasic
            std::cerr << "Centre angle was outside gap. Falling back to CalculateGapCenterAngleBasic" << std::endl;
            gap_center_angle = CalculateGapCenterAngleBasic(*largest_gap);
        }
        Obstacle const & nearest_obstacle = FindNearestObstacle(obstacles);
        // TODO: Set goal angle by some other, reasonable means
        // klapajar: TODO: ^^ Probably set this to make it perpendicular to the line connecting edges of largest gap
        //                 -- Another option is to set this with respect to its current position from car location. (Somehow.)
        final_heading_angle = FollowTheGap::CalculateFinalHeadingAngle(g_goal_angle, gap_center_angle, nearest_obstacle.distance, kGapWeightCoefficient);
    }
    if ( std::isnan(final_heading_angle) ) {
        throw NoGapFoundException("Final heading angle was nan");
    }

    gap_borders_out.emplace_back(
        Obstacle(
            (*largest_gap).obstacle_left->distance_to_center,
            (*largest_gap).obstacle_left->angle,
            (*largest_gap).obstacle_left->radius
        )
    );
    gap_borders_out.emplace_back(
        Obstacle(
            (*largest_gap).obstacle_right->distance_to_center,
            (*largest_gap).obstacle_right->angle,
            (*largest_gap).obstacle_right->radius
        )
    );
    return final_heading_angle;
}

float FollowTheGap::FtgWithAngleFilter(std::vector<Obstacle> obstacles, LidarData * lidar_data, std::vector<Obstacle> & gap_borders_out) {
    // std::vector<Obstacle> obstacles = FollowTheGap::CreateObstaclesWithAngleFilter(lidar_data);
    if ( obstacles.empty() ) {
        std::cerr << "No obstacles found" << std::endl;
        throw NoGapFoundException("No gap found");
    }
    std::vector<Gap>::const_iterator largest_gap;
    std::vector<Gap> gaps;
    float final_heading_angle;
    float gap_angle;
    float const filter_angle_left = lidar_data->angle_min + AngleFilter::left_index*lidar_data->angle_increment;
    float const filter_angle_right = lidar_data->angle_min + AngleFilter::right_index*lidar_data->angle_increment;
    try {
        gaps = FindGapsAngle(obstacles);
        bool ok = false;
        while (!ok && !gaps.empty() ) {
            largest_gap = std::max_element(gaps.cbegin(), gaps.cend(),[](Gap const & a, Gap const & b){
                return a.gap_size < b.gap_size;
            });
            gap_angle = CalculateGapCenterAngle(*largest_gap);
            if ( (gap_angle > filter_angle_right) && (gap_angle < filter_angle_left) ) {
                ok = true;
            } else {
                gaps.erase(largest_gap);
            }
        }
    } catch (InvalidAngleException const & e) {
        std::cerr << "ERROR: Invalid angle encountered when creating gap array: " << std::endl;
        std::cerr << e.what() << std::endl;
        throw NoGapFoundException("Found invalid angle.");
    }

    if ( (largest_gap == gaps.cend()) || gaps.empty() ) {
        throw NoGapFoundException("No gap found");
    }
    else {
        Obstacle const & nearest_obstacle = FindNearestObstacle(obstacles);
        final_heading_angle = FollowTheGap::CalculateFinalHeadingAngle(g_goal_angle, gap_angle, nearest_obstacle.distance, kGapWeightCoefficient);
    }
    if ( std::isnan(final_heading_angle) ) {
        throw NoGapFoundException("Final heading angle was nan");
    }

    gap_borders_out.emplace_back(
        Obstacle(
            (*largest_gap).obstacle_left->distance_to_center,
            (*largest_gap).obstacle_left->angle,
            (*largest_gap).obstacle_left->radius
        )
    );
    gap_borders_out.emplace_back(
        Obstacle(
            (*largest_gap).obstacle_right->distance_to_center,
            (*largest_gap).obstacle_right->angle,
            (*largest_gap).obstacle_right->radius
        )
    );
    return final_heading_angle;
}


float FollowTheGap::FtgWeightedAverageMethod(std::vector<Obstacle> obstacles, LidarData * lidar_data) {
    float const max_range = lidar_data->range_max;
    float const min_range = std::max(lidar_data->range_min, 2.0f);
    float const range_increment = 2;
    int unsigned num_its =(max_range-min_range)/range_increment;
    int unsigned s = 0;
    float final_heading_angle = 0;
    std::vector<Obstacle> obs_out;
    std::vector<Obstacle> gap_borders_out;
    // We add angles together weighted by i, such that the angles from lower kMaxRange get highter weights.
    // The final result is then divided by s, which is the sum of all the weights
    for ( int unsigned i = 1; i <= num_its; ++i ) {
        // kMaxRange = min_range + i*range_increment;
        kMaxRange = max_range - i*range_increment;
        float angle;
        try {
            FollowTheGap::FilterObstacles(obs_out, obstacles);
            angle = FollowTheGapMethod(obs_out, lidar_data, gap_borders_out);
            final_heading_angle += angle * (static_cast<float>(i)/num_its);
            s += i;
        } catch ( NoGapFoundException & e ) {
            // Do nothing
        }
    }
    if ( s == 0 ) {
        throw NoGapFoundException("");
    }
    final_heading_angle /= s;
    return final_heading_angle;
}

float FollowTheGap::FollowTheCornerMethod(std::vector<Obstacle> obstacles, std::vector<Obstacle> & gap_borders_out) {
    Corner corner = FindCorner(obstacles);

    // float final_heading_angle = FindSafeCornerAngle(corner);
    // Try to weight the corner angle with the goal angle
    float const safe_corner_angle = FindSafeCornerAngle(corner);
    Obstacle const & nearest_obstacle = FindNearestObstacle(obstacles);
    float const final_heading_angle = CalculateFinalHeadingAngle(g_goal_angle,
            safe_corner_angle, nearest_obstacle.distance, kCornerWeightCoefficient);

    gap_borders_out.emplace_back(
        Obstacle(
            corner.obstacle_left->distance_to_center,
            corner.obstacle_left->angle,
            corner.obstacle_left->radius
        )
    );
    gap_borders_out.emplace_back(
        Obstacle(
            corner.obstacle_right->distance_to_center,
            corner.obstacle_right->angle,
            corner.obstacle_right->radius
        )
    );
    return final_heading_angle;
}

float FollowTheGap::FollowLargestVerticalGapMethod(std::vector<Obstacle> obstacles, std::vector<Obstacle> & gap_borders_out) {
    float final_heading_angle;
    std::vector<Gap> gaps = FindGapsVerticalDistance(obstacles);
    auto largest_gap = std::max_element(gaps.cbegin(), gaps.cend(),[](Gap const & a, Gap const & b){
        return a.gap_distance < b.gap_distance;
    });
    if ( largest_gap == gaps.cend() ) {
        throw NoGapFoundException("");
    }
    final_heading_angle = FindVerticalGapSafeAngle(*largest_gap);

    gap_borders_out.emplace_back(
        Obstacle(
            (*largest_gap).obstacle_left->distance_to_center,
            (*largest_gap).obstacle_left->angle,
            (*largest_gap).obstacle_left->radius
        )
    );
    gap_borders_out.emplace_back(
        Obstacle(
            (*largest_gap).obstacle_right->distance_to_center,
            (*largest_gap).obstacle_right->angle,
            (*largest_gap).obstacle_right->radius
        )
    );

    return final_heading_angle;
}

std::tuple<bool, float> FollowTheGap::Callback(const std::vector<Obstacle> & obstacles_in, LidarData * lidar_data, std::vector<Obstacle> & obstacles_out, std::vector<Obstacle> & gap_borders_out) {
    float final_heading_angle = 0.0;

    bool ok = false;

    // Try ftg at lower range
    g_fovAngleMax = M_PI/2;

    kMaxRange = 4;
    while ( (!ok) && (kMaxRange>=2) ) {
        try {
            FollowTheGap::FilterObstacles(obstacles_out, obstacles_in);
            final_heading_angle = FollowTheGapMethod(obstacles_out, lidar_data, gap_borders_out);
            ok = true;
        } catch (NoGapFoundException & e) {
            kMaxRange = kMaxRange - 0.5;
        }
    }
    g_fovAngleMax += M_PI/16;

    // Retry ftg for close ranges
    if ( !ok ) {
        kMaxRange = 2;
        g_fovAngleMax = M_PI/2+M_PI/8;
        while ( (!ok) && (kMaxRange>=1.5) ) {
            try {
                FollowTheGap::FilterObstacles(obstacles_out, obstacles_in);
                final_heading_angle = FollowTheGapMethod(obstacles_out, lidar_data, gap_borders_out);
                ok = true;
            } catch (NoGapFoundException & e) {
                kMaxRange = kMaxRange - 0.5;
            }
        }
    }

    // Retry follow the corner for larger angles
    if (!ok) {
        g_fovAngleMax = M_PI/2-M_PI/16;
        while ( (!ok) && (g_fovAngleMax < M_PI) ) {
            kMaxRange = 3;
            while ( (!ok) && (kMaxRange>=0.5) ) {
                try {
                    FollowTheGap::FilterObstacles(obstacles_out, obstacles_in);
                    final_heading_angle = FollowTheCornerMethod(obstacles_out, gap_borders_out);
                    ok = true;
                } catch (NoGapFoundException & e) {
                    kMaxRange = kMaxRange - 0.5;
                }
            }
        g_fovAngleMax += M_PI/32;
        }
    }

    if ( ok ) {
        last_final_heading_angle = final_heading_angle;
    } else {
        std::cerr << "No gaps found" << std::endl;
    }

    return std::make_tuple(ok, final_heading_angle);
}

#if 0
// Currently not used, but prepared for FindGapsAngle
static float FindDistanceBetweenObstacleAndFov(float fov_angle, Obstacle const & obstacle) {
    float gap_size = std::abs(obstacle.angle - fov_angle);
    return obstacle.distance_to_center * std::sin(gap_size) - obstacle.radius;
}

static float FindDistanceBetweenObstacleAndNhol(Obstacle const & obstacle) {
    return obstacle_nhol_left.DistanceBetweenObstacleEdges(obstacle);
}

static float FindDistanceBetweenObstacleAndNholRight(Obstacle const & obstacle) {
    return obstacle_nhol_right.DistanceBetweenObstacleEdges(obstacle);
}

static float FindThetaNhol(Obstacle const & obstacle) {
    // Derivation of this expression was done on paper. See report from pvt spring 2018
    // TODO: Simplify
    double const r = kTurnRadius;
    double const r_squared = r*r;
    double const d_o = obstacle.distance_to_center;
    double const d_o_squared = d_o*d_o;
    double const theta_o = std::abs(obstacle.angle);
    double const d_ro_squared = r_squared + d_o_squared - 2*r*d_o*std::cos(M_PI/2 - theta_o);
    double const d_ro = std::sqrt(d_ro_squared);
    double const theta_r = std::acos((r_squared + d_ro_squared - d_o_squared)/(2*r*d_ro));
    double const d_nhol_to_o = d_ro - r;
    double const d_nhol_to_o_squared = d_nhol_to_o*d_nhol_to_o;
    double const d_r_squared = 2*r_squared*(1-std::cos(theta_r));
    double const d_r = std::sqrt(d_r_squared);
    double const theta = std::acos((d_r_squared + d_o_squared - d_nhol_to_o_squared)/
            (2*d_r*d_o));
    double const theta_nhol = theta + theta_o;
    return (float)theta_nhol;
}
#endif

std::vector<Gap> FollowTheGap::FindGapsAngle(std::vector<Obstacle> & obstacles) {
    std::vector<Gap> gaps;

#if 0
    // TODO
    // First find gap between left border and first obstacle
    float const d_fov_l = FindDistanceBetweenObstacleAndFov(-kFovAngle, obstacles.front());
    float const d_nhol_l = FindDistanceBetweenObstacleAndNhol(obstacles.front());
    float theta_lim_l;
    if ( d_nhol_l >= d_fov_l ) {
        theta_lim_l = -kFovAngle;
    } else {
         float theta_nhol_l = FindThetaNhol(obstacles.front());
        theta_lim_l = theta_nhol_l;
    }
    theta_lim_l = -kFovAngle;
    if ( obstacles.front().angle_left > theta_lim_l ) {
        // There is no gap
    } else {
        // To get the correct distance for calculation of gap center angle, we
        // model the leftmost obstacle as a point at distance d_r
        // with angle theta_lim_l
        if ( std::isnan(theta_lim_l) ) {
            throw InvalidAngleException("theta_lim_l was nan");
        }
        float d_r = std::cos(theta_lim_l)*obstacles.front().distance_to_center;
        obstacle_nhol_left = Obstacle(d_r, theta_lim_l, 0);
        // gaps.emplace_back(obstacle_nhol_left, obstacles.front());
    }
#endif

    // Then find gap between obstacles
    for ( size_t i = 1; i < obstacles.size(); ++i ) {
        if ( obstacles[i-1].Overlaps(obstacles[i]) ) {
            // No gap
            continue;
        }
        else if ( obstacles[i-1].angle_right < obstacles[i].angle_left ) {
            // No gap
            continue;
        }
        gaps.emplace_back(obstacles[i-1], obstacles[i]);
    }

#if 0
    // TODO
    // Finally find gap between last obstacle and right border
    float const d_fov_r = FindDistanceBetweenObstacleAndFov(kFovAngle, obstacles.back());
    float const d_nhol_r = FindDistanceBetweenObstacleAndNhol(obstacles.back());
    float theta_lim_r;
    if ( d_nhol_r >= d_fov_r ) {
        theta_lim_r = kFovAngle;
    } else {
        float const theta_nhol_r = FindThetaNhol(obstacles.back());
        theta_lim_r = theta_nhol_r;
    }
    if ( obstacles.back().angle_right < theta_lim_r ) {
        // There is no gap
    } else {
        // To get the correct distance for calculation of gap center angle, we
        // model the rightmost obstacle as a point at distance d_r
        // with angle theta_lim_r
        if ( std::isnan(theta_lim_r) ) {
            throw InvalidAngleException("theta_lim_r was nan");
        }
        float d_r = std::cos(theta_lim_r)*obstacles.back().distance_to_center;
        obstacle_nhol_right = Obstacle(d_r, theta_lim_r, 0);
        // gaps.emplace_back(obstacles.back(), obstacle_nhol_right);
    }
#endif

    return gaps;
}

Corner FollowTheGap::FindCorner(std::vector<Obstacle> & obstacles) {
    std::vector<Corner> corners;
    for ( size_t i = 1; i < obstacles.size(); ++i ) {
        Obstacle & obstacle_left = obstacles[i-1];
        Obstacle & obstacle_right = obstacles[i];
        float obstacles_distance = obstacle_left.DistanceBetweenObstacleCentres(obstacle_right);
        if ( obstacle_left.x > obstacle_right.x
          && (obstacles_distance > kTrackMinWidth) ) {
            // Found right corner
            corners.emplace_back(obstacle_left, obstacle_right, Corner::CornerTypes::kRight);
        }
        if ( (obstacle_right.x > obstacle_left.x)
          && (obstacles_distance > kTrackMinWidth) ) {
            // Found left corner
            corners.emplace_back(obstacle_left, obstacle_right, Corner::CornerTypes::kLeft);
        }
    }
    if (corners.size() == 1) {
        // We can only accept an unamiguous situation where only one corner was found
        auto largest_corner = std::max_element(corners.cbegin(), corners.cend(), [](Corner const & a, Corner const & b){
                return a.gap_size < b.gap_size;
        });
        return *largest_corner;
    } else {
        throw NoGapFoundException("No corner found");
    }
}

float FollowTheGap::FindSafeCornerAngle(Corner & corner) {
    float angle;
    float theta_d;

    if ( corner.CornerType() == Corner::CornerTypes::kRight ) {
        /* vajnamar: FIXME:  It can happen that we are closer to the corner
         *                   (distance_to_center), than we want the perpendicular
         *                   distance (kDistanceToCorner) to be, then we want
         *                   to compute asin() of values > 1 which leads to a
         *                   NaN result. Temporary fixed by comparing the values
         *                   and forcing theta_d = M_PI/2 for the mentioned case.
         */
        if ( kDistanceToCorner < corner.obstacle_right->distance_to_center )
            theta_d = std::asin(kDistanceToCorner/corner.obstacle_right->distance_to_center);
        else
            theta_d = M_PI/2;
        angle = corner.obstacle_right->angle + theta_d;
        // return corner.obstacle_right->angle_left;
    } else if ( corner.CornerType() == Corner::CornerTypes::kLeft ) {
        if ( kDistanceToCorner < corner.obstacle_left->distance_to_center )
            theta_d = std::asin(kDistanceToCorner/corner.obstacle_left->distance_to_center);
        else
            theta_d = M_PI/2;
        angle = corner.obstacle_left->angle - theta_d;
        // return corner.obstacle_left->angle_right;
    } else {
        // This should not happen
        throw std::runtime_error("FollowTheGap::FindSafeCorner else case not implemented");
    }
    return angle;
}

float FollowTheGap::FindVerticalGapSafeAngle(Gap const & gap) {
    float angle;
    Obstacle const * obstacle_left = gap.obstacle_left;
    Obstacle const * obstacle_right = gap.obstacle_right;
    if ( obstacle_left->distance_to_center > obstacle_right->distance_to_center ) {
        // right obstacle is closer. Implies a right turn, so we must avoid the left side of the right obstacle
        angle = obstacle_right->angle_left;
    } else {
        angle = obstacle_left->angle_right;
    }
    return angle;
}

std::vector<Gap> FollowTheGap::FindGapsVerticalDistance(std::vector<Obstacle> & obstacles) {
    std::vector<Gap> gaps;
    // TODO: First find gap between left border and first obstacle
    // Same as FindGapsAngle

    // Then find gap between obstacles
    for ( size_t i = 1; i < obstacles.size(); ++i ) {
        if ( obstacles[i-1].DistanceBetweenObstacleCentres(obstacles[i]) < kCarRadius ) {
            // Gap too small
            continue;
        }
        gaps.emplace_back(obstacles[i-1], obstacles[i]);
    }

    // TODO: Finally find gap between last obstacle and right border
    // Same as FindGapsAngle

    return gaps;
}

void FilterLoneObstacleGroups(std::vector<Obstacle> & obstacles) {
    float const max_distance_center = 0.15; // Max distance
    for ( auto it = obstacles.begin(); it != obstacles.end();) {
        //Filter first point
        if ( obstacles.begin() == it) {
            if (obstacles.size() > 1)
            {
                auto nx_it = std::next(it, 1);
                if( it->DistanceBetweenObstacleCentres(*nx_it) > max_distance_center ) {
                    it = obstacles.erase(it);
                } else{
                    it++;
                }
            }
            else{
                it++;
            }
        //Filter endpoint
        } else if ( obstacles.end() == it){
            if (obstacles.size() > 1)
            {
                auto pv_it = std::prev(it, 1);
                if( it->DistanceBetweenObstacleCentres(*pv_it) > max_distance_center ) {
                    it = obstacles.erase(it);
                } else{
                    it++;
                }
            }
            else{
                it++;
            }
        //Filter the points that are in the middle
        } else{
            if (obstacles.size() >= 4){
                auto pv_it = std::prev(it, 1);
                auto nx1_it = std::next(it, 1);
                if ( obstacles.end() == nx1_it){
                    if((it->DistanceBetweenObstacleCentres(*pv_it) > max_distance_center) && (it->DistanceBetweenObstacleCentres(*nx1_it) > max_distance_center)){
                        it = obstacles.erase(it);
                    }
                    else{
                        it++;
                    }
                } else{
                   auto nx2_it = std::next(it, 2);
                    if((it->DistanceBetweenObstacleCentres(*pv_it) > max_distance_center) && (it->DistanceBetweenObstacleCentres(*nx1_it) < max_distance_center) && (nx1_it->DistanceBetweenObstacleCentres(*nx2_it) > max_distance_center)) {
                        it = obstacles.erase(it, nx1_it+1);
                    } else if((it->DistanceBetweenObstacleCentres(*pv_it) > max_distance_center) && (it->DistanceBetweenObstacleCentres(*nx1_it) > max_distance_center)){
                        it = obstacles.erase(it);
                    } else{
                        it++;
                    }
                }
            }
            else{
                it++;
            }
        }
    }
}

void FollowTheGap::FilterObstacles(std::vector<Obstacle> & obstacles, const std::vector<Obstacle> & obstacles_in) {
    obstacles.clear();

    for ( auto it = obstacles_in.begin(); it != obstacles_in.end(); ++it) {
        // Filter data which is outside desired fov
        if ( std::abs(it->angle) > g_fovAngleMax ) {
            continue;
        }
        // Filter data which is too far away to be interesting
        else if ( it->distance_to_center > kMaxRange ) {
            continue;
        }
        else {
            obstacles.emplace_back(*it);
        }
    }

    FilterLoneObstacleGroups(obstacles);
}

float FollowTheGap::CalculateGapCenterAngle(Gap const & gap) {
    double const d_1 = gap.obstacle_right->distance;
    double const d_2 = gap.obstacle_left->distance;;
    double const theta_1 = std::abs(gap.angle_right);
    double const theta_2 = std::abs(gap.angle_left);
    double theta_gap_c;

    // The original paper covers only the case where the obstacles are on
    // different sides of the x-axis. Here we describe also the cases
    // where obstacles are on the same side
    if ( (gap.angle_left >= 0) && (gap.angle_right <= 0) ) {
        theta_gap_c = std::acos((d_1 + d_2*std::cos(theta_1+theta_2))/
                    (std::sqrt((d_1*d_1) + (d_2*d_2) + 2*d_1*d_2*std::cos(theta_1+theta_2))))
                - theta_1;
        if ( std::isnan(theta_gap_c) ) {
            throw InvalidAngleException("Gap centre angle was nan for case gap.angle_right <= 0 && gap.angle_left >= 0");
        }
    } else if ( gap.angle_right >= 0 ) {
        // TODO: Simplify
        double const l_squared = (d_1*d_1 + d_2*d_2 - 2*d_1*d_2*std::cos(theta_2 - theta_1))/4;
        double const h_squared = (d_1*d_1 + d_2*d_2 - 2*l_squared) / 2;
        double const h = std::sqrt(h_squared);
        double const theta_x = std::acos((h_squared + d_1*d_1 - l_squared)/(2*h*d_1));
        theta_gap_c = theta_1 + theta_x;
        if ( std::isnan(theta_gap_c) ) {
            throw InvalidAngleException("Gap centre angle was nan for case gap.angle_right >= 0");
        }
    } else {
        // gap.angle_left <= 0
        // TODO: Simplify
        double const l_squared = (d_1*d_1 + d_2*d_2 - 2*d_1*d_2*std::cos(theta_1 - theta_2))/4;
        double const h_squared = (d_1*d_1 + d_2*d_2 - 2*l_squared) / 2;
        double const h = std::sqrt(h_squared);
        double const theta_x = std::acos((h_squared + d_2*d_2 - l_squared)/(2*h*d_2));
        theta_gap_c = -(theta_2 + theta_x);
        if ( std::isnan(theta_gap_c) ) {
            std::stringstream error_msg;
            error_msg << "Gap centre angle was nan for case gap.angle_left <= 0" << std::endl;
            throw InvalidAngleException(error_msg.str());
        }
    }

    if ( (theta_gap_c > gap.angle_left) || (theta_gap_c < gap.angle_right) ) {
        // The centre angle is outside the gap, which should never be the case
        throw CenterOutsideGapException("The calculated centre of gap was outside the gap");
    }

    if ( std::isnan(theta_gap_c) ) {
        throw InvalidAngleException("Gap centre angle was nan unknown case");
    }

    return theta_gap_c;
}

float FollowTheGap::CalculateGapCenterAngleBasic(Gap const & gap) {
    float const theta_1 = gap.angle_right;
    float const theta_2 = gap.angle_left;
    return (theta_1 + theta_2) / 2;
}

float FollowTheGap::CalculateFinalHeadingAngle(float const theta_goal, float const theta_c, float const d_min, float const alpha) {
    // float const & alpha = kGapWeightCoefficient;
    float theta_final = ((alpha/d_min) * theta_c + theta_goal) /
        ((alpha/d_min) + 1);
    return theta_final;
}


