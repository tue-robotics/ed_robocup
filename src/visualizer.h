#ifndef ED_ROBOCUP_VISUALIZER_H_
#define ED_ROBOCUP_VISUALIZER_H_

#include <ed/types.h>
#include <rgbd/types.h>
#include <geolib/datatypes.h>
#include <opencv2/core/core.hpp>
#include <set>

class Visualizer
{

public:

    Visualizer();

    ~Visualizer();

    cv::Mat drawWorldModelOverlay(const ed::WorldModel& world, const rgbd::Image& image,
                                  const geo::Pose3D& sensor_pose);

private:

};

#endif
