#include "navigator.h"

#include <cb_planner_msgs_srvs/LocalPlannerAction.h>
#include <head_ref/HeadReferenceAction.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <ed/entity.h>
#include <ed/world_model.h>

#include <geolib/ros/msg_conversions.h>

#include <ros/node_handle.h>

// ----------------------------------------------------------------------------------------------------

bool getPoint3D(const rgbd::Image& image, int x_depth, int y_depth, geo::Vec3& p_IMAGE)
{
    const cv::Mat& depth = image.getDepthImage();

    if (x_depth < 0 || x_depth >= depth.cols || y_depth < 0 || y_depth >= depth.rows)
    {
        return false;
    }

    float d = depth.at<float>(y_depth, x_depth);
    for(int i = 1; i < 20; ++i)
    {
        for(int x = x_depth - i; x <= x_depth + i; ++x)
        {
            for(int y = y_depth - i; y <= y_depth + i; ++ y)
            {
                if (x < 0 || y < 0 || x >= depth.cols || y >= depth.rows)
                    continue;

                d = depth.at<float>(y, x);
                if (d > 0 && d == d)
                {
                    break;
                }
            }
            if (d > 0 && d == d)
                break;
        }
        if (d > 0 && d == d)
            break;
    }

    if (d <= 0 || d != d)
    {
        return false;
    }

    rgbd::View view(image, depth.cols);
    p_IMAGE = view.getRasterizer().project2Dto3D(x_depth, y_depth) * d;
    return true;
}

// ----------------------------------------------------------------------------------------------------

void Navigator::initialize(ros::NodeHandle& nh, const std::string& nav_goal_topic, const std::string& head_goal_topic)
{
    pub_nav_goal_  = nh.advertise<cb_planner_msgs_srvs::LocalPlannerActionGoal>(nav_goal_topic, 1);
    pub_head_goal_ = nh.advertise<head_ref::HeadReferenceActionGoal>(head_goal_topic, 1);
}

// ----------------------------------------------------------------------------------------------------

bool Navigator::navigate(const rgbd::Image& image, const geo::Pose3D& sensor_pose, int click_x, int click_y)
{
    geo::Vec3 p_SENSOR;
    if (!getPoint3D(image, click_x, click_y, p_SENSOR))
        return false;

    geo::Vec3 p_MAP = sensor_pose * p_SENSOR;

    // Determine navigation goal
    geo::Vec3 robot_pos = sensor_pose.t;
    robot_pos.z = 0;

    double wanted_dist_to_goal = 2.0;

    p_MAP.z = 0;
    geo::Vec3 lookat_dir = (p_MAP - robot_pos).normalized();

    // Construct orientation matrix from direction
    geo::Mat3 ori = geo::Mat3::identity();
    ori.xx = lookat_dir.x;
    ori.yx = lookat_dir.y;
    ori.xy = -lookat_dir.y;
    ori.yy = lookat_dir.x;

    std::vector<geo::Pose3D> path;
    path.push_back(geo::Pose3D(ori, robot_pos));

    if (false) // For now, only rotate
    {
        // Create plan for navigation

        geo::Vec3 goal_pos = (p_MAP - wanted_dist_to_goal * lookat_dir);

        double dist_to_goal = (goal_pos - robot_pos).length();
        geo::Vec3 goal_dir = (goal_pos - robot_pos) / dist_to_goal;

        double waypoint_dist = 0.1;
        int num_waypoints = dist_to_goal / waypoint_dist;

        for(int i = 0; i < num_waypoints; ++i)
        {
            geo::Pose3D& p = path.back();
            path.push_back(geo::Pose3D(ori, p.t + waypoint_dist * goal_dir));
        }
        path.push_back(geo::Pose3D(ori, goal_pos));
    }

    // -----------------------------------------------------------
    // Fill goal message

    // orientation
    cb_planner_msgs_srvs::LocalPlannerActionGoal goal_msg;
    goal_msg.goal.orientation_constraint.frame = "/map";

    geo::convert(p_MAP, goal_msg.goal.orientation_constraint.look_at);

    // path
    goal_msg.goal.plan.resize(path.size());
    for(unsigned int i = 0; i < path.size(); ++i)
    {
        geo::convert(path[i], goal_msg.goal.plan[i].pose);
        goal_msg.goal.plan[i].header.frame_id = "/map";
    }

    // -----------------------------------------------------------
    // Publish goal

    pub_nav_goal_.publish(goal_msg);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool Navigator::moveHead(const rgbd::Image& image, const geo::Pose3D& sensor_pose,
                         const ed::WorldModel& world, int click_x, int click_y)
{
    geo::Vec3 p_SENSOR;
    if (!getPoint3D(image, click_x, click_y, p_SENSOR))
        return false;

    geo::Vec3 p_MAP = sensor_pose * p_SENSOR;

    head_ref::HeadReferenceActionGoal goal_msg;
    goal_msg.goal.goal_type = head_ref::HeadReferenceGoal::LOOKAT;


    // TODO: make this nice

    ed::EntityConstPtr sergio = world.getEntity("sergio");
    ed::EntityConstPtr amigo = world.getEntity("amigo");
    if (sergio)
    {
        geo::Vec3 p_BASE_LINK = sergio->pose().inverse() * p_MAP;
        geo::convert(p_BASE_LINK, goal_msg.goal.target_point.point);
        goal_msg.goal.target_point.header.frame_id = "/sergio/base_link";
    }
    else if (amigo)
    {
        geo::Vec3 p_BASE_LINK = amigo->pose().inverse() * p_MAP;
        geo::convert(p_BASE_LINK, goal_msg.goal.target_point.point);
        goal_msg.goal.target_point.header.frame_id = "/amigo/base_link";
    }
    else
    {
        geo::convert(p_MAP, goal_msg.goal.target_point.point);
        goal_msg.goal.target_point.header.frame_id = "/map";
    }

    goal_msg.goal.target_point.header.stamp = ros::Time(image.getTimestamp());
    goal_msg.goal.priority = 2;

    pub_head_goal_.publish(goal_msg);
}

// ----------------------------------------------------------------------------------------------------

