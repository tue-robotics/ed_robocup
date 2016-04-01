#ifndef ED_ROBOCUP_PLUGIN_H_
#define ED_ROBOCUP_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/types.h>

#include <ed/kinect/image_buffer.h>
#include <ed/kinect/fitter.h>

#include <tf/transform_listener.h>

#include "ed_robocup/FitEntityInImage.h"
#include "ed_robocup/GetModelImages.h"
#include <std_srvs/Empty.h>

#include <rgbd/GetRGBD.h>

// Map filtering
#include "map_filter.h"

// Model loading
#include <ed/models/model_loader.h>

#include "visualizer.h"

// ----------------------------------------------------------------------------------------------------

struct EntityModel
{
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

//    std::string topic_;

    ImageBuffer image_buffer_;

    ros::CallbackQueue cb_queue_;


    // MODELS

    ed::models::ModelLoader model_loader_;

    std::map<std::string, EntityModel> models_;


    // FITTING

    Fitter fitter_;


    // Map fitering

    double wall_height_;

    ros::Time last_wall_creation_;

    MapFilter map_filter_;


    // DATA

    const ed::WorldModel* world_;

    ed::UpdateRequest* update_req_;


    // VISUALIZATION

    Visualizer visualizer_;


    // SERVICES

    ros::ServiceServer srv_fit_entity_;

    bool srvFitEntityInImage(ed_robocup::FitEntityInImage::Request& req, ed_robocup::FitEntityInImage::Response& res);

    ros::ServiceServer srv_get_model_images_;

    bool srvGetModelImages(ed_robocup::GetModelImages::Request& req, ed_robocup::GetModelImages::Response& res);

    ros::ServiceServer srv_create_walls_;

    bool srvCreateWalls(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    ros::ServiceServer srv_get_image_;

    bool srvGetImage(rgbd::GetRGBD::Request& req, rgbd::GetRGBD::Response& res);
};

#endif
