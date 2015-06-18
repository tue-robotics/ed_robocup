#include "plugin.h"

#include <rgbd/Image.h>
#include <geolib/ros/tf_conversions.h>

#include <ed/update_request.h>
#include <ed/entity.h>
#include <ed/measurement.h>

// ----------------------------------------------------------------------------------------------------

RobocupPlugin::RobocupPlugin() : tf_listener_(0)
{
}

// ----------------------------------------------------------------------------------------------------

RobocupPlugin::~RobocupPlugin()
{
    delete tf_listener_;
}

// ----------------------------------------------------------------------------------------------------

void RobocupPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    if (config.value("topic", topic_))
    {
        std::cout << "[ED KINECT PLUGIN] Initializing kinect client with topic '" << topic_ << "'." << std::endl;
        kinect_client_.intialize(topic_);
    }

    if (config.readGroup("segmentation", tue::REQUIRED))
    {
        config.value("max_correspondence_distance", segmenter_.association_correspondence_distance_);
        config.value("downsample_factor", segmenter_.downsample_factor_);
        config.value("max_range", segmenter_.max_range_);
        config.endGroup();
    }

    tf_listener_ = new tf::TransformListener;

    // Initialize image publishers for visualization
    viz_segmentation_.initialize("viz/segmentation");

    // Initialize lock entity server
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue_);

    srv_segment_ = nh.advertiseService("segment", &RobocupPlugin::srvSegment, this);
    srv_classify_ = nh.advertiseService("classify", &RobocupPlugin::srvClassify, this);
}

// ----------------------------------------------------------------------------------------------------

void RobocupPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
{
    world_ = &data.world;
    update_req_ = &req;

    // - - - - - - - - - - - - - - - - - -
    // Check ROS callback queue

    cb_queue_.callAvailable();    
}

// ----------------------------------------------------------------------------------------------------

bool RobocupPlugin::srvSegment(ed_robocup::Segment::Request& req, ed_robocup::Segment::Response& res)
{
    // - - - - - - - - - - - - - - - - - -
    // Fetch kinect image and place in image buffer

    rgbd::ImageConstPtr rgbd_image = kinect_client_.nextImage();
    if (!rgbd_image)
    {
        ROS_WARN("[ED ROBOCUP] No RGBD image available");
        return true;
    }

    // - - - - - - - - - - - - - - - - - -
    // Determine absolute kinect pose based on TF

    geo::Pose3D sensor_pose;

    if (!tf_listener_->waitForTransform("map", rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), ros::Duration(1.0)))
    {
        ROS_WARN("[ED ROBOCUP] Could not get camera pose");
        return true;
    }

    try
    {
        tf::StampedTransform t_sensor_pose;
        tf_listener_->lookupTransform("map", rgbd_image->getFrameId(), ros::Time(rgbd_image->getTimestamp()), t_sensor_pose);
        geo::convert(t_sensor_pose, sensor_pose);
    }
    catch(tf::TransformException& ex)
    {
        ROS_WARN("[ED ROBOCUP] Could not get camera pose: %s", ex.what());
        return true;
    }

    // Convert from ROS coordinate frame to geolib coordinate frame
    sensor_pose.R = sensor_pose.R * geo::Matrix3(1, 0, 0, 0, -1, 0, 0, 0, -1);

    std::vector<Segment> segments;
    segmenter_.segment(*world_, *rgbd_image, sensor_pose, segments);

    for(std::vector<Segment>::const_iterator it = segments.begin(); it != segments.end(); ++it)
    {
        const Segment& cluster = *it;

        ed::UUID id;
        ed::ConvexHull new_chull;
        geo::Pose3D new_pose;

        // No assignment, so add as new cluster
        new_chull = cluster.chull;
        new_pose = cluster.pose;

        // Generate unique ID
        id = ed::Entity::generateID();

        // Update existence probability
        update_req_->setExistenceProbability(id, 1); // TODO magic number

        // Set convex hull and pose
        if (!new_chull.points.empty())
        {
            update_req_->setConvexHullNew(id, new_chull, new_pose, rgbd_image->getTimestamp(), rgbd_image->getFrameId());
        }

        // Set timestamp
        update_req_->setLastUpdateTimestamp(id, rgbd_image->getTimestamp());

        // Add measurement
        update_req_->addMeasurement(id, ed::MeasurementConstPtr(new ed::Measurement(rgbd_image, cluster.mask, sensor_pose)));

        res.new_entities.push_back(id.str());
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool RobocupPlugin::srvClassify(ed_robocup::Classify::Request& req,ed_robocup::Classify::Response& res)
{
    return true;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(RobocupPlugin)
