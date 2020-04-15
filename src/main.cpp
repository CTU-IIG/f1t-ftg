/*
 *
 * Main, ROS-enabled layer
 *
 * Authors: Jaroslav Klapálek
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
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"

ros::NodeHandle * node_handle_pointer;

int unsigned kPublishMessageBufferSize = 10;
int unsigned kSubscribeMessageBufferSize = 1;

ros::Publisher publisher_final_heading_angle;

// For visualizing and debugging
ros::Publisher publisher_visualize_largest_gap;
ros::Publisher publisher_visualize_final_heading_angle;
ros::Publisher publisher_visualize_obstacles;


//////////////////////
// Utility functions
//////////////////////

std::vector<FollowTheGap::Obstacle> CreateObstacles(sensor_msgs::LaserScan::ConstPtr const & lidar_data) {
    size_t const data_size = lidar_data->ranges.size();
    float const & angle_increment = lidar_data->angle_increment;
    float const & range_max = lidar_data->range_max;
    float const & range_min = lidar_data->range_min;
    float angle = lidar_data->angle_min;
    std::vector<FollowTheGap::Obstacle> obstacles;
    for ( size_t i = 0; i < data_size; ++i, angle += angle_increment ) {
        float const & range = lidar_data->ranges[i];
        // Throw away all range values that are NAN
        /* Sometimes LIDARs return NAN / inf values */
        if (std::isnan(range)) {
            continue;
        }
        // Filter data which is outside the lidar fov
        if ( (angle < lidar_data->angle_min ) || (angle > lidar_data->angle_max) ) {
            continue;
        }
        // Filter data which is outside the max range or inside min range of lidar
        if ( (range < range_min) || (range > range_max) ) {
            continue;
        }
        // Throw away all outliers (point without neighbours)
        // klapajar: TODO: It is possible to use "is someone around me?" but it leads to losing information about space
        //                 It seems to be better using vectors. but TODO.
        float obstacle_radius = FollowTheGap::kCarRadius;
        obstacles.emplace_back(range, angle, obstacle_radius);
    }
    // We reverse the array so that the first obstacles are on the left(largest angle, in accordance with the paper)
    std::reverse(obstacles.begin(), obstacles.end());
    return obstacles;
}

std::vector<FollowTheGap::Obstacle> CreateObstaclesWithAngleFilter(
        sensor_msgs::LaserScan::ConstPtr const & lidar_data) {
    size_t const data_size = lidar_data->scan_time/lidar_data->time_increment;
    float const & angle_increment = lidar_data->angle_increment;
    float const & range_max = lidar_data->range_max;
    float const & range_min = lidar_data->range_min;
    // We start at the angle corresponding to the right index
    float angle = lidar_data->angle_min + FollowTheGap::AngleFilter::right_index*angle_increment;
    std::vector<FollowTheGap::Obstacle> obstacles;
    std::cerr << "right_index: " << FollowTheGap::AngleFilter::right_index << std::endl;
    std::cerr << "left_index: " << FollowTheGap::AngleFilter::left_index << std::endl;
    for ( size_t i = FollowTheGap::AngleFilter::right_index; i <= FollowTheGap::AngleFilter::left_index; ++i, angle += angle_increment ) {
        float const & range = lidar_data->ranges[i];
        // Throw away all range values that are NAN
        /* Sometimes LIDARs return NAN / inf values */
        if (std::isnan(range)) {
            continue;
        }
        // Filter data which is outside the lidar fov
        if ( (angle < lidar_data->angle_min ) || (angle > lidar_data->angle_max) ) {
            continue;
        }
        // Filter data which is outside the max range or inside min range of lidar
        if ( (range < range_min) || (range > range_max) ) {
            continue;
        }
        // Throw away all outliers (point without neighbours)
        // klapajar: TODO: It is possible to use "is someone around me?" but it leads to losing information about space
        //                 It seems to be better using vectors. but TODO.
        float obstacle_radius = FollowTheGap::kCarRadius;
        obstacles.emplace_back(range, angle, obstacle_radius);
    }
    // We reverse the array so that the first obstacles are on the left(largest angle, in accordance with the paper)
    std::reverse(obstacles.begin(), obstacles.end());
    return obstacles;
}


//////////////////////
// Publishers
//////////////////////

void PublishFinalHeadingAngle(float final_heading_angle) {
    std_msgs::Float32 final_heading_angle_message;
    final_heading_angle_message.data = final_heading_angle;
    publisher_final_heading_angle.publish(final_heading_angle_message);
}

void PublishVisualizeFinalHeadingAngle(float final_heading_angle) {
    geometry_msgs::PoseStamped angle_message;
    angle_message.header.frame_id = "/robot_0/base_laser_link";

    tf::Quaternion angle_quaternion;
    angle_quaternion.setEuler(0, 0, final_heading_angle);
    angle_quaternion.normalize();
    tf::quaternionTFToMsg(angle_quaternion, angle_message.pose.orientation);

    publisher_visualize_final_heading_angle.publish(angle_message);
}

void PublishVisualizeLargestGap(FollowTheGap::Obstacle const & gap_left, FollowTheGap::Obstacle const & gap_right) {

    geometry_msgs::PointStamped p0, p1, robot_point;
    robot_point.header.frame_id = "/robot_0/base_laser_link";
    robot_point.point.x = 0;
    robot_point.point.y = 0;
    p0.point.x = gap_left.distance*std::cos(gap_left.angle_right);
    p0.point.y = gap_left.distance*std::sin(gap_left.angle_right);
    p0.header.frame_id = "/robot_0/base_laser_link";
    p1.point.x = gap_right.distance*std::cos(gap_right.angle_left);
    p1.point.y = gap_right.distance*std::sin(gap_right.angle_left);
    p1.header.frame_id = "/robot_0/base_laser_link";

    publisher_visualize_largest_gap.publish(robot_point);
    publisher_visualize_largest_gap.publish(p0);
    publisher_visualize_largest_gap.publish(p1);

}

void PublishVisualizeObstacles(std::vector<FollowTheGap::Obstacle> const & obstacles) {
    visualization_msgs::Marker obstacle_points;
    obstacle_points.header.frame_id = "/robot_0/base_laser_link";
    obstacle_points.header.stamp = ros::Time::now();
    obstacle_points.type = visualization_msgs::Marker::POINTS;

    // obstacle_points.scale.x = kCarRadius;
    // obstacle_points.scale.y = kCarRadius;
    obstacle_points.scale.x = 0.05;
    obstacle_points.scale.y = 0.05;
    obstacle_points.color.r = 0;
    obstacle_points.color.g = 1;
    obstacle_points.color.b = 0;
    obstacle_points.color.a = 1;

    for ( auto const & o : obstacles ) {
        geometry_msgs::Point p;
        p.x = o.x;
        p.y = o.y;
        obstacle_points.points.push_back(p);
    }

    publisher_visualize_obstacles.publish(obstacle_points);
}


//////////////////////
// Callbacks
//////////////////////

void Callback(sensor_msgs::LaserScan::ConstPtr const & lidar_data) {
    LidarData ld(lidar_data->range_min,
                                 lidar_data->range_max,
                                 lidar_data->angle_min,
                                 lidar_data->angle_max,
                                 lidar_data->angle_increment
                    );

    std::vector<FollowTheGap::Obstacle> obstacles = CreateObstacles(lidar_data);


    bool ok;
    float angle;
    std::vector<FollowTheGap::Obstacle> obstacles_out;
    std::vector<FollowTheGap::Obstacle> gap_borders_out;

    std::tie(ok, angle) = FollowTheGap::Callback(obstacles, &ld, obstacles_out, gap_borders_out);

    std::cout << ok << ", " << angle << std::endl;

    if (ok) {
        PublishFinalHeadingAngle(angle);
        PublishVisualizeFinalHeadingAngle(angle);
        PublishVisualizeLargestGap(gap_borders_out.at(0), gap_borders_out.at(1));
    }

    PublishVisualizeObstacles(obstacles_out);
}

void AngleFilterLeftCallback(std_msgs::Int32::ConstPtr const & message) {
    FollowTheGap::AngleFilter::left_index = message->data;
}

void AngleFilterRightCallback(std_msgs::Int32::ConstPtr const & message) {
    FollowTheGap::AngleFilter::right_index = message->data;
}

void GoalAngleCallback(std_msgs::Float64::ConstPtr const & message) {
    FollowTheGap::g_goal_angle = message->data;
}


//////////////////////
// Main
//////////////////////

int main(int argc, char ** argv) {
    ros::init(argc, argv, "follow_the_gap");
    node_handle_pointer = new ros::NodeHandle;


    // Subscribers
    ros::Subscriber lidar_data_subscriber = node_handle_pointer
        ->subscribe("/scan", kSubscribeMessageBufferSize,
                Callback
    );

    ros::Subscriber angle_filter_subscriber_right = node_handle_pointer
        ->subscribe("/right_constraint_index", kSubscribeMessageBufferSize,
                AngleFilterRightCallback
    );

    ros::Subscriber angle_filter_subscriber_left = node_handle_pointer
        ->subscribe("/left_constraint_index", kSubscribeMessageBufferSize,
                AngleFilterLeftCallback
    );

    ros::Subscriber goal_angle_subscriber = node_handle_pointer
        ->subscribe("/lsr/angle", kSubscribeMessageBufferSize,
                GoalAngleCallback
    );


    // Publishers
    publisher_final_heading_angle = node_handle_pointer
        ->advertise<std_msgs::Float32>("/final_heading_angle", kPublishMessageBufferSize);

    publisher_visualize_largest_gap = node_handle_pointer
        ->advertise<geometry_msgs::PointStamped>("/visualize_largest_gap", kPublishMessageBufferSize);

    publisher_visualize_final_heading_angle = node_handle_pointer
        ->advertise<geometry_msgs::PoseStamped>("/visualize_final_heading_angle", kPublishMessageBufferSize);

    publisher_visualize_obstacles = node_handle_pointer
        ->advertise<visualization_msgs::Marker>("/visualize_obstacles", kPublishMessageBufferSize);

    while(ros::ok()) {
        ros::spin();
    }
    return 0;
}
