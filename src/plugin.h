#ifndef ED_ROBOCUP_PLUGIN_H_
#define ED_ROBOCUP_PLUGIN_H_

#include <queue>

#include <ed/plugin.h>
#include <ed/types.h>
#include <ed/helpers/image_publisher.h>

#include <rgbd/Client.h>

#include <tf/transform_listener.h>

#include "ed/convex_hull.h"

// Locking
#include "ed_robocup/Segment.h"
#include "ed_robocup/Classify.h"

#include "segmentation.h"

// ----------------------------------------------------------------------------------------------------

class RobocupPlugin : public ed::Plugin
{

public:

    RobocupPlugin();

    ~RobocupPlugin();

    void initialize(ed::InitData& init);

    void process(const ed::PluginInput& data, ed::UpdateRequest& req);

private:

    Segmenter segmenter_;

    // VISUALIZATION

    ed::ImagePublisher viz_segmentation_;


    // COMMUNICATION

    std::string topic_;

    rgbd::Client kinect_client_;

    tf::TransformListener* tf_listener_;

    ros::CallbackQueue cb_queue_;


    // DATA

    const ed::WorldModel* world_;

    ed::UpdateRequest* update_req_;


    // SERVICES

    ros::ServiceServer srv_segment_;

    bool srvSegment(ed_robocup::Segment::Request& req, ed_robocup::Segment::Response& res);

    ros::ServiceServer srv_classify_;

    bool srvClassify(ed_robocup::Classify::Request& req,ed_robocup::Classify::Response& res);

};

#endif
