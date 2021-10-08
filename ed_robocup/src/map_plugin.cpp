#include "map_plugin.h"

#include <rgbd/image.h>
#include <rgbd/view.h>

#include <geolib/ros/tf_conversions.h>

#include <ed/update_request.h>
#include <ed/entity.h>
#include <ed/measurement.h>
#include <ed/world_model.h>
#include <ed/entity.h>

#include <opencv2/highgui/highgui.hpp>

#include "timer.h"

// ----------------------------------------------------------------------------------------------------

RobocupMapPlugin::RobocupMapPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

RobocupMapPlugin::~RobocupMapPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void RobocupMapPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    std::string map_topic_in, map_topic_out;
    if (config.value("map_topic_in", map_topic_in))
    {
        config.value("map_topic_out", map_topic_out);
        map_filter_.initialize(map_topic_in, map_topic_out);
    }

    if (!config.value("wall_height", wall_height_, tue::config::OPTIONAL))
        wall_height_ = 0.8;

    last_wall_creation_ = ros::Time(0);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Initialize services
    ros::NodeHandle nh_global;
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue_);

    if (!config.value("map_filter_padding", map_filter_padding_, tue::config::OPTIONAL))
        map_filter_padding_ = 0.3;

    srv_create_walls_ = nh.advertiseService("create_walls", &RobocupMapPlugin::srvCreateWalls, this);
}

// ----------------------------------------------------------------------------------------------------

void RobocupMapPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    world_ = &data.world;
    update_req_ = &req;

    // - - - - - - - - - - - - - - - - - -
    // Check ROS callback queue

    cb_queue_.callAvailable();

    // -------------------------------------
    // Map filter

    map_filter_.update();

//    if (ros::Time::now() - last_wall_creation_ > ros::Duration(3))
//    {
//        geo::ShapeConstPtr shape = map_filter_.createWallShape(wall_height_);

//        if (shape)
//        {
//            ed::UUID id = "walls";

//            update_request_->setShape(id, shape);
//            update_request_->setPose(id, geo::Pose3D::identity());
//        }

//        last_wall_creation_ = ros::Time::now();
//    }
}

// ----------------------------------------------------------------------------------------------------

bool RobocupMapPlugin::srvCreateWalls(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ROS_INFO("[ED] RobocupMapPlugin: CreateWalls requested");

    geo::ShapeConstPtr shape = map_filter_.createWallShape(wall_height_);

    if (shape)
    {
        ed::UUID id = "walls";

        update_req_->setShape(id, shape);
        update_req_->setPose(id, geo::Pose3D::identity());
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(RobocupMapPlugin)
