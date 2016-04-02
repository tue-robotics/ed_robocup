#include "navigator.h"

#include <cb_planner_msgs_srvs/LocalPlannerAction.h>

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <ed/entity.h>
#include <ed/world_model.h>

#include <geolib/ros/msg_conversions.h>

#include <ros/node_handle.h>

// ----------------------------------------------------------------------------------------------------

void Navigator::initialize(ros::NodeHandle& nh, const std::string& goal_topic)
{
    pub_goal_ = nh.advertise<cb_planner_msgs_srvs::LocalPlannerActionGoal>(goal_topic, 1);
}

// ----------------------------------------------------------------------------------------------------

bool Navigator::navigate(const rgbd::Image& image, const geo::Pose3D& sensor_pose, int click_x, int click_y)
{
    const cv::Mat& depth = image.getDepthImage();

    if (click_x < 0 || click_x >= depth.cols || click_y < 0 || click_y >= depth.rows)
    {
        return false;
    }

    float d = depth.at<float>(click_y, click_x);
    for(int i = 1; i < 20; ++i)
    {
        for(int x = click_x - i; x <= click_x + i; ++x)
        {
            for(int y = click_y - i; y <= click_y + i; ++ y)
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
    geo::Vec3 p_SENSOR = view.getRasterizer().project2Dto3D(click_x, click_y) * d;

    geo::Vec3 p_MAP = sensor_pose * p_SENSOR;

    // Determine navigation goal
    geo::Vec3 robot_pos = sensor_pose.t;
    robot_pos.z = 0;

    double wanted_dist_to_goal = 2.0;

    p_MAP.z = 0;
    geo::Vec3 lookat_dir = (p_MAP - robot_pos).normalized();
    geo::Vec3 goal_pos = (p_MAP - wanted_dist_to_goal * lookat_dir);

    double dist_to_goal = (goal_pos - robot_pos).length();
    geo::Vec3 goal_dir = (goal_pos - robot_pos) / dist_to_goal;

    // Construct orientation matrix from direction
    geo::Mat3 ori = geo::Mat3::identity();
    ori.xx = lookat_dir.x;
    ori.yx = lookat_dir.y;
    ori.xy = -lookat_dir.y;
    ori.yy = lookat_dir.x;

    std::vector<geo::Pose3D> path;
    path.push_back(geo::Pose3D(ori, robot_pos));

    double waypoint_dist = 0.1;
    int num_waypoints = dist_to_goal / waypoint_dist;

    for(int i = 0; i < num_waypoints; ++i)
    {
        geo::Pose3D& p = path.back();
        path.push_back(geo::Pose3D(ori, p.t + waypoint_dist * goal_dir));
    }
    path.push_back(geo::Pose3D(ori, goal_pos));

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

    pub_goal_.publish(goal_msg);

    return true;
}

// ----------------------------------------------------------------------------------------------------

