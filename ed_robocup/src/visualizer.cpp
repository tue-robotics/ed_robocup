#include "visualizer.h"

#include <rgbd/image.h>
#include <rgbd/view.h>

#include <ed/world_model.h>
#include <ed/entity.h>

#include <geolib/Shape.h>

#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

class SimpleRenderResult : public geo::RenderResult
{

public:

    SimpleRenderResult(cv::Mat& z_buffer_, cv::Mat& entity_index_map_)
        : geo::RenderResult(z_buffer_.cols, z_buffer_.rows), z_buffer(z_buffer_),
          entity_index_map(entity_index_map_) {}

    void renderPixel(int x, int y, float depth, int /*i_triangle*/)
    {
        float old_depth = z_buffer.at<float>(y, x);
        if (old_depth == 0 || depth < old_depth)
        {
            z_buffer.at<float>(y, x) = depth;
            entity_index_map.at<int>(y, x) = i_entity;
        }
    }

    cv::Mat& z_buffer;
    int i_entity;
    cv::Mat& entity_index_map;

};

// ----------------------------------------------------------------------------------------------------

Visualizer::Visualizer()
{
}

// ----------------------------------------------------------------------------------------------------

Visualizer::~Visualizer()
{
}

// ----------------------------------------------------------------------------------------------------

cv::Mat Visualizer::drawWorldModelOverlay(const ed::WorldModel& world, const rgbd::Image& image,
                                          const geo::Pose3D& sensor_pose, int image_width)
{
    std::set<std::string> ignore_ids;
    ignore_ids.insert("floor");
    ignore_ids.insert("walls");

    const cv::Mat& rgb_orig = image.getRGBImage();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Create resized image

    cv::Mat rgb;
    if (image_width > 0)
    {
        double ratio_rgb = (double)image_width / rgb_orig.cols;
        cv::resize(rgb_orig, rgb, cv::Size(image_width, rgb_orig.rows * ratio_rgb));
    }
    else
    {
        rgb = rgb_orig.clone();
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - -

    image_width = rgb.cols;
    int image_height = rgb.rows;

    rgbd::View view(image, image_width);

    cv::Mat depth_render(image_height, image_width, CV_32FC1, 0.0);
    cv::Mat entity_index_map(image_height, image_width, CV_32SC1, cv::Scalar(-1));

    SimpleRenderResult res(depth_render, entity_index_map);

    int i_entity = 0;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Render depth image based on world model

    for(ed::WorldModel::const_iterator it = world.begin(); it != world.end(); ++it)
    {
        const ed::EntityConstPtr& e = *it;

        if (!e->shape() || !e->has_pose() || ignore_ids.find(e->id().str()) != ignore_ids.end())
            continue;

        geo::RenderOptions opt;
        opt.setMesh(e->shape()->getMesh(), sensor_pose.inverse() * e->pose());

        res.i_entity = i_entity++;

        view.getRasterizer().render(opt, res);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Draw world model on top of background

    cv::Mat canvas = rgb.clone();

    for(int y = 0; y < canvas.rows - 1; ++y)
    {
        for(int x = 0; x < canvas.cols - 1; ++x)
        {
            int i1 = entity_index_map.at<int>(y, x);
            int i2 = entity_index_map.at<int>(y, x + 1);
            int i3 = entity_index_map.at<int>(y + 1, x);

            if (i1 != i2 || i1 != i3)
            {
                // Entity edge
                canvas.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 255);
            }
            else if (i1 >= 0)
            {
                // Entity surface
                cv::Vec3b& c = canvas.at<cv::Vec3b>(y, x);
                c[0] = std::min(255, 2 * c[0]);
                c[1] = 100;
                c[2] = 100;
            }
        }
    }

    return canvas;
}

// ----------------------------------------------------------------------------------------------------
