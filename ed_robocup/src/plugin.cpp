#include "plugin.h"

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

// Decomposes 'pose' into a (X, Y, YAW) and (Z, ROLL, PITCH) component
void decomposePose(const geo::Pose3D& pose, geo::Pose3D& pose_xya, geo::Pose3D& pose_zrp)
{
    tf::Matrix3x3 m;
    geo::convert(pose.R, m);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    pose_xya.R.setRPY(0, 0, yaw);
    pose_xya.t = geo::Vec3(pose.t.x, pose.t.y, 0);

    pose_zrp = pose_xya.inverse() * pose;
}

// ----------------------------------------------------------------------------------------------------

bool ImageToMsg(const cv::Mat& image, const std::string& encoding, ed_robocup_msgs::NamedImage& msg)
{
    msg.encoding = encoding;

    cv::Mat rgb_image;
    if (image.channels() == 1)
    {
        // depth image
        rgb_image = cv::Mat(image.rows, image.cols, CV_8UC3, cv::Scalar(0, 0, 0));
        for(unsigned int i = 0; i < rgb_image.rows * rgb_image.cols; ++i)
            rgb_image.at<cv::Vec3b>(i) = (image.at<float>(i) / 10) * cv::Vec3b(255, 255, 255);
    }
    else
    {
        rgb_image = image;
    }

    if (encoding == "jpg")
    {
        // OpenCV compression settings
        std::vector<int> rgb_params;
        rgb_params.resize(3, 0);

        rgb_params[0] = cv::IMWRITE_JPEG_QUALITY;
        rgb_params[1] = 50; // default is 95

        // Compress image
        if (!cv::imencode(".jpg", rgb_image, msg.data, rgb_params))
        {
            std::cout << "RGB image compression failed" << std::endl;
            return false;
        }
    }
    else if (encoding == "png")
    {
        std::vector<int> params;
        params.resize(3, 0);

        params[0] = cv::IMWRITE_PNG_COMPRESSION;
        params[1] = 1;

        if (!cv::imencode(".png", rgb_image, msg.data, params)) {
            std::cout << "PNG image compression failed" << std::endl;
            return false;
        }
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

RobocupPlugin::RobocupPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

RobocupPlugin::~RobocupPlugin()
{
}

// ----------------------------------------------------------------------------------------------------

void RobocupPlugin::initialize(ed::InitData& init)
{
    tue::Configuration& config = init.config;

    // Initialize RGBD client
    std::string rgbd_topic;
    if (config.value("rgbd_topic", rgbd_topic))
        image_buffer_.initialize(rgbd_topic, "map");

    std::string map_topic_in, map_topic_out;
    if (config.value("map_topic_in", map_topic_in))
    {
        config.value("map_topic_out", map_topic_out);
        map_filter_.initialize(map_topic_in, map_topic_out);
    }

    if (!config.value("wall_height", wall_height_, tue::config::OPTIONAL))
        wall_height_ = 0.8;

    // Load models (used for fitting)
    if (config.readArray("models"))
    {
        while(config.nextArrayItem())
        {
            std::string id, type;
            if (!config.value("id", id) | !config.value("type", type))
                continue;

            ed::WorldModel world_model;

            // Load model data
            ed::UpdateRequest req;
            std::stringstream error;

            if (!model_loader_.create(id, type, req, error, true))
            {
                ed::log::error() << "While loading model '" << type << "': " << error.str() << std::endl;
                continue;
            }

            world_model.update(req);
            ed::EntityConstPtr entity = world_model.getEntity(id);

            if (!entity)
            {
                ed::log::error() << "While loading model '" << type << "': could not load model (this should never happen)" << std::endl;
                continue;
            }

            if (!entity->shape())
            {
                ed::log::error() << "While loading model '" << type << "': model does not have a shape" << std::endl;
                continue;
            }

            cv::Mat model_image;
            std::string image_path;
            if (config.value("image", image_path, tue::config::OPTIONAL))
            {
                model_image = cv::imread(image_path);

                if (!model_image.data)
                    ed::log::error() << "Could not load model image: '" << image_path << "'." << std::endl;
            }

            if (!model_image.data)
            {
                model_image = cv::Mat(200, 200, CV_8UC3, cv::Scalar(0, 0, 0));
            }

            EntityModel& model = models_[id];
            model.entity_prototype = entity;
            model.model_image = model_image;
            model.type = type;
            model.id = id;
        }

        config.endArray();
    }

    last_wall_creation_ = ros::Time(0);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Initialize services
    ros::NodeHandle nh_global;
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue_);

    std::string nav_goal_topic;
    std::string head_goal_topic;
    if (config.value("nav_goal_topic", nav_goal_topic) && config.value("head_goal_topic", head_goal_topic))
        navigator_.initialize(nh_global, nav_goal_topic, head_goal_topic);

    if (!config.value("map_filter_padding", map_filter_padding_, tue::config::OPTIONAL))
        map_filter_padding_ = 0.3;

    srv_fit_entity_ = nh.advertiseService("fit_entity_in_image", &RobocupPlugin::srvFitEntityInImage, this);
    srv_get_model_images_ = nh.advertiseService("get_model_images", &RobocupPlugin::srvGetModelImages, this);
    srv_create_walls_ = nh.advertiseService("create_walls", &RobocupPlugin::srvCreateWalls, this);
    srv_get_image_ = nh.advertiseService("get_image", &RobocupPlugin::srvGetImage, this);
}

// ----------------------------------------------------------------------------------------------------

void RobocupPlugin::process(const ed::PluginInput& data, ed::UpdateRequest& req)
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

bool RobocupPlugin::srvFitEntityInImage(ed_robocup_msgs::FitEntityInImage::Request& req, ed_robocup_msgs::FitEntityInImage::Response& res)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check for undo

    if (req.undo_latest_fit)
    {
        if (fitted_ids_.empty())
        {
            res.error_msg = "Nothing to undo";
            return true;
        }

        ed::UUID entity_id = fitted_ids_.back();
        update_req_->removeEntity(entity_id);
        fitted_ids_.pop_back();
        return true;
    }

    ROS_INFO("[ED] RobocupPlugin: FitEntityInImage requested");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Capture camera image

    rgbd::ImageConstPtr image;
    geo::Pose3D sensor_pose;

    if (!image_buffer_.waitForRecentImage(image, sensor_pose, 1.0))
    {
        res.error_msg = "Could not capture image";
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Determine click location

    int x = req.px * image->getDepthImage().cols;
    int y = req.py * image->getDepthImage().rows;

    if (req.entity_type == "NAVIGATE")
    {
        // Navigate
        navigator_.moveHead(*image, sensor_pose, *world_, x, y);
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check if model exists

    std::map<std::string, EntityModel>::const_iterator it_model = models_.find(req.entity_type);
    if (it_model == models_.end())
    {
        res.error_msg = "Unknown entity_type: '" + req.entity_type + "'";
        return true;
    }

    const EntityModel& model = it_model->second;
    ed::UUID entity_id = model.id;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Add object (but with incorrect location)

    std::stringstream error;
    if (!model_loader_.create(entity_id, model.type, *update_req_, error, true))
    {
        res.error_msg = "Could not spawn entity";
        return true;
    }
    //Set correct type, so not inheriting from other types
    update_req_->setType(entity_id, model.type);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Set location based on initial click

    rgbd::View view(*image, image->getDepthImage().cols);

    // Decompose the sensor_pose into (x, y, yaw) and (z, roll, pitch)
    geo::Pose3D sensor_pose_xya, sensor_pose_zrp;
    decomposePose(sensor_pose, sensor_pose_xya, sensor_pose_zrp);

    // Estimate based on the pixel of the entity annotation where it is w.r.t.
    // the sensor
    geo::Pose3D pose_SENSOR_XYA;
    pose_SENSOR_XYA.t = geo::Vec3(view.getRasterizer().project2Dto3DX(x), 1, 0);

    pose_SENSOR_XYA.R.setRPY(0, 0, -0.5 * M_PI); // This assumes estimated entity position is with its x-axis towards camera

    // Calculate the entity pose in map frame
    geo::Pose3D pose_MAP = sensor_pose_xya * pose_SENSOR_XYA;
    pose_MAP.t.z = 0;

    update_req_->setPose(entity_id, pose_MAP);

    ed::WorldModel world_model_tmp = *world_;
    world_model_tmp.update(*update_req_);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Update the entity pose using fitting

    if (!fitter_.isConfigured())
    {
        fitter_.configureBeamModel(image->getCameraModel());
    }

    FitterData fitter_data;
    fitter_.processSensorData(*image, sensor_pose, fitter_data);

    geo::Pose3D fitted_pose;
    if (!fitter_.estimateEntityPose(fitter_data, world_model_tmp, entity_id, pose_MAP, fitted_pose, 0.5 * M_PI))
    {
        res.error_msg = "Could not fit entity";
        return true;
    }

    update_req_->setPose(entity_id, fitted_pose);

    {
        ed::WorldModel wm = *world_;
        wm.update(*update_req_);
        ed::EntityConstPtr e = wm.getEntity(entity_id);

        EntityRepresentation2D repr2d = fitter_.GetOrCreateEntity2D(e);
        if (!repr2d.shape_2d.empty())
        {
            geo::Transform2 pose_2d;
            pose_2d.t = geo::Vec2(fitted_pose.t.x, fitted_pose.t.y);
            pose_2d.R = geo::Mat2(fitted_pose.R.xx, fitted_pose.R.xy,
                                  fitted_pose.R.yx, fitted_pose.R.yy);
            map_filter_.setEntityPose(pose_2d, repr2d.shape_2d, map_filter_padding_);
        }
    }

    fitted_ids_.push_back(entity_id);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool RobocupPlugin::srvGetModelImages(ed_robocup_msgs::GetModelImages::Request& req, ed_robocup_msgs::GetModelImages::Response& res)
{
    ROS_INFO("[ED] RobocupPlugin: GetModelImages requested");

    res.models.resize(models_.size());

    unsigned int i = 0;
    for(std::map<std::string, EntityModel>::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const EntityModel& model = it->second;

        ed_robocup_msgs::NamedImage& model_msg = res.models[i];
        model_msg.name = it->first;

        ImageToMsg(model.model_image, "jpg", model_msg);

        ++i;
    }

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool RobocupPlugin::srvCreateWalls(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    ROS_INFO("[ED] RobocupPlugin: CreateWalls requested");

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

bool RobocupPlugin::srvGetImage(rgbd_msgs::GetRGBD::Request& req, rgbd_msgs::GetRGBD::Response& res)
{
    Timer timer;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check for valid input

    if (req.compression != rgbd_msgs::GetRGBD::Request::JPEG && req.compression != rgbd_msgs::GetRGBD::Request::PNG)
    {
        ROS_ERROR("Invalid compression, only JPEG and PNG are supported (see ENUM in srv definition)");
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Grab camera image

    rgbd::ImageConstPtr image;
    geo::Pose3D sensor_pose;

    if (!image_buffer_.nextImage(image, sensor_pose))
    {
        ROS_DEBUG("Could not capture image");
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Draw image on top

    cv::Mat canvas = visualizer_.drawWorldModelOverlay(*world_, *image, sensor_pose, req.width);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Compress image

    // Compress images
    std::string compression_str = req.compression == rgbd_msgs::GetRGBD::Request::JPEG ? ".jpeg" : ".png";
    if (cv::imencode(compression_str, canvas, res.rgb_data))
    {

//        ROS_INFO_STREAM("[ED] RobocupPlugin: srvGetImage took " << timer.getElapsedTimeInMilliSec()
//                        << " ms, sending " << res.rgb_data.size() << " byte");
        return true;
    }

    ROS_ERROR_STREAM("cv::imencode with compression_str " << compression_str << " failed!");


    return true;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(RobocupPlugin)
