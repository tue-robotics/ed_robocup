#ifndef ED_ROBOCUP_NAVIGATOR_H_
#define ED_ROBOCUP_NAVIGATOR_H_

#include <rgbd/types.h>
#include <ed/types.h>
#include <geolib/datatypes.h>

#include <ros/publisher.h>

class Navigator
{

public:

    void initialize(ros::NodeHandle& nh, const std::string& goal_topic);

    bool navigate(const rgbd::Image& image, const geo::Pose3D& sensor_pose, int click_x, int click_y);

private:

    ros::Publisher pub_goal_;

};

#endif
