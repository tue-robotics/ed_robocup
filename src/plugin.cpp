#include "plugin.h"

#include <rgbd/Image.h>
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

    // Initialize RGBD client
    std::string rgbd_topic;
    if (config.value("rgbd_topic", rgbd_topic))
        rgbd_client_.intialize(rgbd_topic);

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

    tf_listener_ = new tf::TransformListener;

    last_wall_creation_ = ros::Time(0);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Initialize services
    ros::NodeHandle nh("~");
    nh.setCallbackQueue(&cb_queue_);

    srv_fit_entity_ = nh.advertiseService("fit_entity_in_image", &RobocupPlugin::srvFitEntityInImage, this);
    srv_get_model_images_ = nh.advertiseService("get_model_images", &RobocupPlugin::srvGetModelImages, this);
    srv_create_walls_ = nh.advertiseService("create_walls", &RobocupPlugin::srvCreateWalls, this);
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


    return true;
}

// ----------------------------------------------------------------------------------------------------

bool RobocupPlugin::srvGetModelImages(ed_robocup::GetModelImages::Request& req, ed_robocup::GetModelImages::Response& res)
{
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

ED_REGISTER_PLUGIN(RobocupPlugin)
