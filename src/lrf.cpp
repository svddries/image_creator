#include "lrf.h"
#include "world_model.h"

#include <opencv2/imgproc/imgproc.hpp>

// ----------------------------------------------------------------------------------------------------

std::vector<double> renderLRF(const geo::LaserRangeFinder& lrf, const geo::Transform2& lrf_pose, const WorldModel2D& wm)
{
    geo::Transform2 lrf_pose_inv = lrf_pose.inverse();

    std::vector<double> ranges(lrf.getNumBeams(), 0);

    for(unsigned int i = 0; i < wm.entities.size(); ++i)
    {
        const Entity2D& e = wm.entities[i];
        geo::Transform2 t = lrf_pose_inv * e.pose;

        for(std::vector<Contour2D>::const_iterator it = e.shape.contours.begin(); it != e.shape.contours.end(); ++it)
        {
            const Contour2D& c = *it;

            for(unsigned int j = 0; j < c.points.size(); ++j)
            {
                geo::Vec2 p1 = t * c.points[j];
                geo::Vec2 p2 = t * c.points[(j + 1) % c.points.size()];
                lrf.renderLine(p1, p2, ranges);
            }
        }
    }

    return ranges;
}

// ----------------------------------------------------------------------------------------------------

void rangesToImagePoints(Canvas& canvas, geo::LaserRangeFinder& lrf, const geo::Transform2& lrf_pose, const std::vector<double>& ranges,
                         std::vector<cv::Point>& points_image)
{
    points_image.resize(ranges.size());

    for(unsigned int i = 0; i < ranges.size(); ++i)
    {
        double r = ranges[i];
        if (r <= 0)
            continue;

        const geo::Vec3& ray_dir = lrf.getRayDirection(i);
        geo::Vec2 p = lrf_pose * geo::Vec2(r * ray_dir.x, r * ray_dir.y);
        points_image[i] = canvas.worldToImage(p);
    }
}

// ----------------------------------------------------------------------------------------------------

void drawRanges(Canvas& canvas, const geo::LaserRangeFinder& lrf, const geo::Transform2& lrf_pose, const std::vector<double>& ranges,
                const Color& point_color, const Color& line_color)
{
    cv::Point p_lrf = canvas.worldToImage(lrf_pose.t);

    for(unsigned int i = 0; i < ranges.size(); ++i)
    {
        double r = ranges[i];
        if (r <= 0)
            continue;

        const geo::Vec3& ray_dir = lrf.getRayDirection(i);

        geo::Vec2 p = lrf_pose * geo::Vec2(r * ray_dir.x, r * ray_dir.y);

        cv::Point p_cv = canvas.worldToImage(p);
        cv::circle(canvas.image, p_cv, point_color.thickness, point_color.color, CV_FILLED);

        if (line_color.valid)
            cv::line(canvas.image, p_lrf, p_cv, line_color.color, line_color.thickness);
    }
}

// ----------------------------------------------------------------------------------------------------
