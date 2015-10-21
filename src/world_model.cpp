#include "world_model.h"

#include "canvas.h"

#include <opencv2/imgproc/imgproc.hpp>

// ----------------------------------------------------------------------------------------------------

Model2D createBox(double width, double height, bool inside_out)
{
    return createBox(geo::Vec2(-width / 2, -height / 2), geo::Vec2(width / 2, height / 2), inside_out);
}

// ----------------------------------------------------------------------------------------------------

Model2D createBox(const geo::Vec2& p1, const geo::Vec2& p2, bool inside_out)
{
//    geo::Vec2 hsize = p2 - p1;
//    geo::Vec2 pos = (p1 + p2) / 2;
//    std::cout << "wm.addEntity(createBox(" << hsize.x << ", " << hsize.y << "), fromXYADegrees(" << pos.x << ", " << pos.y << ", 0));" << std::endl;

    Model2D model;
    Contour2D& c = model.addContour();
    createBoxContour(p1, p2, c, inside_out);
    return model;
}

// ----------------------------------------------------------------------------------------------------

void createBoxContour(const geo::Vec2& p1, const geo::Vec2& p2, Contour2D& c, bool inside_out)
{
    double x_min = std::min(p1.x, p2.x);
    double x_max = std::max(p1.x, p2.x);
    double y_min = std::min(p1.y, p2.y);
    double y_max = std::max(p1.y, p2.y);

    if (inside_out)
    {
        c.addPoint(x_min, y_min);
        c.addPoint(x_max, y_min);
        c.addPoint(x_max, y_max);
        c.addPoint(x_min, y_max);
    }
    else
    {
        c.addPoint(x_min, y_min);
        c.addPoint(x_min, y_max);
        c.addPoint(x_max, y_max);
        c.addPoint(x_max, y_min);
    }
}

// ----------------------------------------------------------------------------------------------------

void createCircleContour(double radius, Contour2D& c, int num_corners)
{
    for(int i = 0; i < num_corners; ++i)
    {
        double a = 6.283 * i / num_corners;
        c.addPoint(sin(a) * radius, cos(a) * radius);
    }
}

// ----------------------------------------------------------------------------------------------------

Model2D createCircle(double radius, int num_corners)
{
    Model2D model;
    Contour2D& c = model.addContour();
    createCircleContour(radius, c, num_corners);
    return model;
}

// ----------------------------------------------------------------------------------------------------

geo::Transform2 fromXYA(double x, double y, double a)
{
    geo::Transform2 t;
    t.t = geo::Vec2(x, y);
    t.setRotation(a);
    return t;
}

geo::Transform2 fromXYADegrees(double x, double y, double a_degrees)
{
    return fromXYA(x, y, a_degrees / 180 * M_PI);
}

// ----------------------------------------------------------------------------------------------------

void drawModel(Canvas& canvas, const Model2D& m, const geo::Transform2& m_pose, const Color& color)
{
    for(std::vector<Contour2D>::const_iterator it = m.contours.begin(); it != m.contours.end(); ++it)
    {
        const Contour2D& c = *it;

        for(unsigned int j = 0; j < c.points.size(); ++j)
        {
            cv::Point p1_img = canvas.worldToImage(m_pose * c.points[j]);
            cv::Point p2_img = canvas.worldToImage(m_pose * c.points[(j + 1) % c.points.size()]);

            cv::line(canvas.image, p1_img, p2_img, color.color, color.thickness, CV_AA);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void drawWorld(Canvas& canvas, const WorldModel2D& wm)
{
    for(unsigned int i = 0; i < wm.entities.size(); ++i)
    {
        const Entity2D& e = wm.entities[i];
        drawModel(canvas, e.shape, e.pose, e.color);
    }
}

// ----------------------------------------------------------------------------------------------------
