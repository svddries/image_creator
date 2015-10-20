#ifndef _CANVAS_H_
#define _CANVAS_H_

#include <opencv2/core/core.hpp>
#include <geolib/datatypes.h>

// ----------------------------------------------------------------------------------------------------

struct Color
{
    Color() : valid(false) {}
    Color(int r, int g, int b, int thickness_ = 1) : valid(true), color(b, g, r), thickness(thickness_) {}

    bool valid;
    cv::Scalar color;
    int thickness;
};

// ----------------------------------------------------------------------------------------------------

struct Canvas
{
    Canvas(int width_, int height_, const cv::Scalar& background_color_)
        : image(height_, width_, CV_8UC3, background_color_), background_color(background_color_), pixels_per_meter(80)
    {
        center = 0.5 * cv::Point(image.cols, image.rows);
    }

    void clear()
    {
        image.setTo(background_color);
    }

    cv::Point worldToImage(const geo::Vec2& p_world) const
    {
        return cv::Point(p_world.x * pixels_per_meter + center.x,
                         p_world.y * pixels_per_meter + center.y);

    }

    bool worldToImage(const geo::Vec2& p_world, cv::Point& p_image) const
    {
        p_image.x =  p_world.x * pixels_per_meter + center.x;
        p_image.y =  p_world.y * pixels_per_meter + center.y;

        return (p_image.x >= 0 && p_image.y >= 0 && p_image.x < image.cols && p_image.y < image.rows);
    }

    geo::Vec2 imageToWorld(const cv::Point& p_image) const
    {
        return geo::Vec2((p_image.x - center.x) / pixels_per_meter,
                         (p_image.y - center.y) / pixels_per_meter);
    }

    Canvas createSubCanvas(double x, double y, double width, double height)
    {
        int xp = x * image.cols;
        int yp = y * image.cols;
        int wp = width * image.cols;
        int hp = height * image.cols;

        Canvas sub(wp, hp, background_color);
        sub.image = image(cv::Rect(xp, yp, wp, hp));
        sub.clear();

        int border = 10;
        cv::Mat roi = image(cv::Rect(xp - border, yp - border, wp + border * 2, hp + border * 2));
        roi.setTo(background_color);
        cv::rectangle(roi, cv::Point(border / 2, border / 2), cv::Point(roi.cols - border / 2, roi.rows - border / 2), cv::Scalar(100, 100, 100), 2);

        return sub;
    }

    int width() const { return image.cols; }

    int height() const { return image.rows; }

    cv::Mat image;

    cv::Scalar background_color;

    double pixels_per_meter;
    cv::Point center;
};

#endif
