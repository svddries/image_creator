#ifndef IMAGE_CREATOR_WORLD_MODEL_H_
#define IMAGE_CREATOR_WORLD_MODEL_H_

#include "canvas.h"

// ----------------------------------------------------------------------------------------------------

struct Contour2D
{
    std::vector<geo::Vec2> points;

    void addPoint(double x, double y) { points.push_back(geo::Vec2(x, y)); }
};

// ----------------------------------------------------------------------------------------------------

struct Model2D
{
    std::vector<Contour2D> contours;
    Contour2D& addContour()
    {
        contours.push_back(Contour2D());
        return contours.back();
    }
};

// ----------------------------------------------------------------------------------------------------

struct Entity2D
{
    Entity2D() {}
    Entity2D(const Model2D& shape_, const geo::Transform2& pose_, const Color& color_)
        : shape(shape_), pose(pose_), color(color_) {}

    Model2D shape;
    geo::Transform2 pose;
    Color color;
};

// ----------------------------------------------------------------------------------------------------


struct WorldModel2D
{
    std::vector<Entity2D> entities;

    void addEntity(const Model2D& m, const geo::Transform2& t, const Color& color = Color(0, 0, 0, 2))
    {
        entities.push_back(Entity2D(m, t, color));
    }

    WorldModel2D createTransformed(const geo::Transform2& t)
    {
        WorldModel2D wm_t = *this;
        for(unsigned int i = 0; i < wm_t.entities.size(); ++i)
        {
            Entity2D& e = wm_t.entities[i];
            e.pose = t * e.pose;
        }

        return wm_t;
    }

};

// ----------------------------------------------------------------------------------------------------

void createBoxContour(const geo::Vec2& p1, const geo::Vec2& p2, Contour2D& c, bool inside_out = false);

Model2D createBox(double width, double height, bool inside_out = false);

Model2D createBox(const geo::Vec2& p1, const geo::Vec2& p2, bool inside_out = false);

void createCircleContour(double radius, Contour2D& c, int num_corners = 20);

Model2D createCircle(double radius, int num_corners = 20);

// ----------------------------------------------------------------------------------------------------

geo::Transform2 fromXYA(double x, double y, double a);

geo::Transform2 fromXYADegrees(double x, double y, double a_degrees);

// ----------------------------------------------------------------------------------------------------

void drawModel(Canvas& canvas, const Model2D& m, const geo::Transform2& m_pose, const Color& color);

// ----------------------------------------------------------------------------------------------------

void drawWorld(Canvas& canvas, const WorldModel2D& wm);

#endif
