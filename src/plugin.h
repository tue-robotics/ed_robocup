#ifndef ED_ROBOCUP_PLUGIN_H_
#define ED_ROBOCUP_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/types.h>

#include <rgbd/Client.h>

#include <tf/transform_listener.h>

#include "ed_robocup/FitEntityInImage.h"
#include "ed_robocup/GetModelImages.h"

// Map filtering
#include "map_filter.h"

// Model loading
#include <ed/models/model_loader.h>

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

    std::string topic_;

    rgbd::Client rgbd_client_;

    tf::TransformListener* tf_listener_;

    ros::CallbackQueue cb_queue_;


    // MODELS

    ed::models::ModelLoader model_loader_;

    std::map<std::string, EntityModel> models_;


    // Map fitering

    double wall_height_;

    ros::Time last_wall_creation_;

    MapFilter map_filter_;


    // DATA

    const ed::WorldModel* world_;

    ed::UpdateRequest* update_req_;


    // SERVICES

    ros::ServiceServer srv_fit_entity_;

    bool srvFitEntityInImage(ed_robocup::FitEntityInImage::Request& req, ed_robocup::FitEntityInImage::Response& res);

    ros::ServiceServer srv_get_model_images_;

    bool srvGetModelImages(ed_robocup::GetModelImages::Request& req, ed_robocup::GetModelImages::Response& res);

};

#endif
