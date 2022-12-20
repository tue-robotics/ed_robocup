#ifndef ED_ROBOCUP_PLUGIN_H_
#define ED_ROBOCUP_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/types.h>

#include <ed/kinect/fitter.h>

#include <rgbd/image_buffer/image_buffer.h>

#include <tf/transform_listener.h>

#include "ed_robocup_msgs/FitEntityInImage.h"
#include "ed_robocup_msgs/GetModelImages.h"
#include <std_srvs/Empty.h>

#include <rgbd_msgs/GetRGBD.h>

// Map filtering
#include "map_filter.h"

// Model loading
#include <ed/models/model_loader.h>

#include "navigator.h"
#include "visualizer.h"

// ----------------------------------------------------------------------------------------------------

struct EntityModel
{
    std::string id;
    std::string type;
    ed::EntityConstPtr entity_prototype;
    cv::Mat model_image;
};

// ----------------------------------------------------------------------------------------------------

class RobocupPlugin : public ed::Plugin
{

public:

    RobocupPlugin();

    ~RobocupPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    // COMMUNICATION

    rgbd::ImageBuffer image_buffer_;

    ros::CallbackQueue cb_queue_;


    // MODELS

    ed::models::ModelLoader model_loader_;

    std::map<std::string, EntityModel> models_;


    // FITTING

    Fitter fitter_;

    // Stack with fitted ids used for undoing
    std::vector<ed::UUID> fitted_ids_;


    // Map fitering

    double wall_height_;

    ros::Time last_wall_creation_;

    MapFilter map_filter_;

    double map_filter_padding_;


    // NAVIGATION

    Navigator navigator_;


    // DATA

    const ed::WorldModel* world_;

    ed::UpdateRequest* update_req_;


    // VISUALIZATION

    Visualizer visualizer_;


    // SERVICES

    ros::ServiceServer srv_fit_entity_;

    bool srvFitEntityInImage(ed_robocup_msgs::FitEntityInImage::Request& req, ed_robocup_msgs::FitEntityInImage::Response& res);

    ros::ServiceServer srv_get_model_images_;

    bool srvGetModelImages(ed_robocup_msgs::GetModelImages::Request& req, ed_robocup_msgs::GetModelImages::Response& res);

    ros::ServiceServer srv_create_walls_;

    bool srvCreateWalls(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    ros::ServiceServer srv_get_image_;

    bool srvGetImage(rgbd_msgs::GetRGBD::Request& req, rgbd_msgs::GetRGBD::Response& res);
};

#endif
