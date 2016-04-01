#include "plugin.h"

#include <rgbd/Image.h>
#include <rgbd/View.h>

#include <geolib/ros/tf_conversions.h>

#include <ed/update_request.h>
#include <ed/entity.h>
#include <ed/measurement.h>
#include <ed/world_model.h>
#include <ed/entity.h>

#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

bool ImageToMsg(const cv::Mat& image, const std::string& encoding, ed_robocup::NamedImage& msg)
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

        rgb_params[0] = CV_IMWRITE_JPEG_QUALITY;
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

        params[0] = CV_IMWRITE_PNG_COMPRESSION;
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
        image_buffer_.initialize(rgbd_topic);

    std::string map_topic_in, map_topic_out;
    if (config.value("map_topic_in", map_topic_in))
    {
        config.value("map_topic_out", map_topic_out);
        map_filter_.initialize(map_topic_in, map_topic_out);
    }

    if (!config.value("wall_height", wall_height_, tue::OPTIONAL))
        wall_height_ = 0.8;

    // Load models (used for fitting)
    if (config.readArray("models"))
    {
        while(config.nextArrayItem())
        {
            std::string name;
            if (!config.value("name", name))
                continue;

            ed::WorldModel world_model;

            // Load model data
            ed::UpdateRequest req;
            std::stringstream error;
            ed::UUID tmp_id = "id";

            if (!model_loader_.create(tmp_id, name, req, error))
            {
                ed::log::error() << "While loading model '" << name << "': " << error.str() << std::endl;
                continue;
            }

            world_model.update(req);
            ed::EntityConstPtr entity = world_model.getEntity(tmp_id);

            if (!entity)
            {
                ed::log::error() << "While loading model '" << name << "': could not load model (this should never happen)" << std::endl;
                continue;
            }

            if (!entity->shape())
            {
                ed::log::error() << "While loading model '" << name << "': model does not have a shape" << std::endl;
                continue;
            }

            cv::Mat model_image;
            std::string image_path;
            if (config.value("image", image_path, tue::OPTIONAL))
            {
                model_image = cv::imread(image_path);

                if (!model_image.data)
                    ed::log::error() << "Could not load model image: '" << image_path << "'." << std::endl;
            }

            if (!model_image.data)
            {
                model_image = cv::Mat(200, 200, CV_8UC3, cv::Scalar(0, 0, 0));
            }

            EntityModel& model = models_[name];
            model.entity_prototype = entity;
            model.model_image = model_image;
        }

        config.endArray();
    }

    last_wall_creation_ = ros::Time(0);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Initialize services
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue_);

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
}

// ----------------------------------------------------------------------------------------------------

bool RobocupPlugin::srvFitEntityInImage(ed_robocup::FitEntityInImage::Request& req, ed_robocup::FitEntityInImage::Response& res)
{
    ROS_INFO("[ED] RobocupPlugin: FitEntityInImage requested");

    rgbd::ImageConstPtr image;
    geo::Pose3D sensor_pose;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Capture camera image

    if (!image_buffer_.waitForRecentImage("/map", image, sensor_pose, 1.0))
    {
        res.error_msg = "Could not capture image";
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

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Add object (but with incorrect location)

    std::stringstream error;
    ed::UUID entity_id = req.entity_type + "-0";

    if (!model_loader_.create(entity_id, req.entity_type, *update_req_, error))
    {
        res.error_msg = "Could not spawn entity";
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Set location based on initial click

    int x = req.px * image->getDepthImage().cols;
    int y = req.py * image->getDepthImage().rows;
    rgbd::View view(*image, image->getDepthImage().cols);

    geo::Vec3 pos = sensor_pose * (view.getRasterizer().project2Dto3D(x, y) * 3);
    pos.z = 0;

    geo::Pose3D init_entity_pose(geo::Mat3::identity(), pos);

    update_req_->setPose(entity_id, init_entity_pose);

    ed::WorldModel world_model_tmp = *world_;
    world_model_tmp.update(*update_req_);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Update the entity pose using fitting

    FitterData fitter_data;
    fitter_.processSensorData(*image, sensor_pose, fitter_data);

    geo::Pose3D fitted_pose;
    if (!fitter_.estimateEntityPose(fitter_data, world_model_tmp, entity_id, init_entity_pose, fitted_pose, 0.5 * M_PI))
    {
        res.error_msg = "Could not fit entity";
        return true;
    }

    update_req_->setPose(entity_id, fitted_pose);

    return true;
}

// ----------------------------------------------------------------------------------------------------

bool RobocupPlugin::srvGetModelImages(ed_robocup::GetModelImages::Request& req, ed_robocup::GetModelImages::Response& res)
{
    ROS_INFO("[ED] RobocupPlugin: GetModelImages requested");

    res.models.resize(models_.size());

    unsigned int i = 0;
    for(std::map<std::string, EntityModel>::const_iterator it = models_.begin(); it != models_.end(); ++it)
    {
        const EntityModel& model = it->second;

        ed_robocup::NamedImage& model_msg = res.models[i];
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

bool RobocupPlugin::srvGetImage(rgbd::GetRGBD::Request& req, rgbd::GetRGBD::Response& res)
{
    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Check for valid input

    if (req.compression != rgbd::GetRGBD::Request::JPEG && req.compression != rgbd::GetRGBD::Request::PNG)
    {
        ROS_ERROR("Invalid compression, only JPEG and PNG are supported (see ENUM in srv definition)");
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Grab camera image

    rgbd::ImageConstPtr image;
    geo::Pose3D sensor_pose;

    if (!image_buffer_.nextImage("/map", image, sensor_pose))
    {
        ROS_ERROR("Could not capture image");
        return true;
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Draw image on top

    cv::Mat canvas = visualizer_.drawWorldModelOverlay(*world_, *image, sensor_pose);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Create resized image

    cv::Mat resized_canvas;
    double ratio_rgb = 1;
    if (req.width > 0)
        ratio_rgb = (double) req.width / canvas.cols;

    cv::resize(canvas, resized_canvas, cv::Size(req.width, canvas.rows * ratio_rgb));

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Compress image

    // Compress images
    std::string compression_str = req.compression == rgbd::GetRGBD::Request::JPEG ? ".jpeg" : ".png";
    if (cv::imencode(compression_str, resized_canvas, res.rgb_data))
        return true;

    ROS_ERROR_STREAM("cv::imencode with compression_str " << compression_str << " failed!");

    return true;
}

// ----------------------------------------------------------------------------------------------------

ED_REGISTER_PLUGIN(RobocupPlugin)
