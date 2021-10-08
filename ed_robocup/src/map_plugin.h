#ifndef ED_ROBOCUP_PLUGIN_H_
#define ED_ROBOCUP_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/types.h>

#include <tf/transform_listener.h>

#include <std_srvs/Empty.h>

// Map filtering
#include "map_filter.h"

// Model loading
#include <ed/models/model_loader.h>

// ----------------------------------------------------------------------------------------------------

class RobocupMapPlugin : public ed::Plugin
{

public:

    RobocupMapPlugin();

    ~RobocupMapPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    // COMMUNICATION

//    std::string topic_;

    ros::CallbackQueue cb_queue_;


    // Map fitering

    double wall_height_;

    ros::Time last_wall_creation_;

    MapFilter map_filter_;

    double map_filter_padding_;

    // DATA

    const ed::WorldModel* world_;

    ed::UpdateRequest* update_req_;


    // SERVICES

    ros::ServiceServer srv_create_walls_;

    bool srvCreateWalls(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
};

#endif
