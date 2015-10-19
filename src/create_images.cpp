#include <geolib/sensors/LaserRangeFinder.h>
#include <opencv2/highgui/highgui.hpp>

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

// ----------------------------------------------------------------------------------------------------

struct Contour2D
{
    std::vector<geo::Vec2> points;

    void addPoint(double x, double y) { points.push_back(geo::Vec2(x, y)); }
};

struct Model2D
{
    std::vector<Contour2D> contours;
    Contour2D& addContour()
    {
        contours.push_back(Contour2D());
        return contours.back();
    }
};

struct WorldModel2D
{
    std::vector<Model2D> entities;
    std::vector<geo::Transform2> poses;

    void addEntity(const Model2D& m, const geo::Transform2& t)
    {
        entities.push_back(m);
        poses.push_back(t);
    }

};

Model2D createBox(const geo::Vec2& p1, const geo::Vec2& p2, bool inside_out = false)
{
    Model2D model;
    Contour2D& c = model.addContour();

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

    return model;
}

// ----------------------------------------------------------------------------------------------------

Model2D createCircle(double radius, int num_corners = 20)
{
    Model2D model;
    Contour2D& c = model.addContour();

    // Calculate vertices
    for(int i = 0; i < num_corners; ++i)
    {
        double a = 6.283 * i / num_corners;
        c.addPoint(sin(a) * radius, cos(a) * radius);
    }

    return model;
}

// ----------------------------------------------------------------------------------------------------

Model2D createParticle()
{
    Model2D model = createCircle(0.1);
    Contour2D& c = model.addContour();

    c.addPoint(0, 0);
    c.addPoint(0.2, 0);

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

// ----------------------------------------------------------------------------------------------------

void drawModel(Canvas& canvas, const Model2D& m, const geo::Transform2& m_pose, const cv::Scalar& color, int thickness = 1)
{
    for(std::vector<Contour2D>::const_iterator it = m.contours.begin(); it != m.contours.end(); ++it)
    {
        const Contour2D& c = *it;

        for(unsigned int j = 0; j < c.points.size(); ++j)
        {
            cv::Point p1_img = canvas.worldToImage(m_pose * c.points[j]);
            cv::Point p2_img = canvas.worldToImage(m_pose * c.points[(j + 1) % c.points.size()]);

            cv::line(canvas.image, p1_img, p2_img, color, thickness);
        }
    }
}

// ----------------------------------------------------------------------------------------------------

void drawWorld(Canvas& canvas, const WorldModel2D& wm)
{
    for(unsigned int i = 0; i < wm.entities.size(); ++i)
    {
        const Model2D& m = wm.entities[i];
        const geo::Transform2& m_pose = wm.poses[i];

        drawModel(canvas, m, m_pose, cv::Scalar(0, 0, 0), 2);
    }
}

// ----------------------------------------------------------------------------------------------------

std::vector<double> renderLRF(const geo::LaserRangeFinder& lrf, const geo::Transform2& lrf_pose, const WorldModel2D& wm)
{
    geo::Transform2 lrf_pose_inv = lrf_pose.inverse();

    std::vector<double> ranges(lrf.getNumBeams(), 0);

    for(unsigned int i = 0; i < wm.entities.size(); ++i)
    {
        const Model2D& m = wm.entities[i];
        const geo::Transform2& m_pose = wm.poses[i];

        geo::Transform2 t = lrf_pose_inv * m_pose;

        for(std::vector<Contour2D>::const_iterator it = m.contours.begin(); it != m.contours.end(); ++it)
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

void drawRanges(Canvas& canvas, const geo::LaserRangeFinder& lrf, const geo::Transform2& lrf_pose, const std::vector<double>& ranges)
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
        cv::circle(canvas.image, p_cv, 2, cv::Scalar(0, 0, 255), CV_FILLED);
        cv::line(canvas.image, p_lrf, p_cv, cv::Scalar(0, 0, 255));
    }
}

// ----------------------------------------------------------------------------------------------------

void drawParticleFilter(Canvas& canvas, const geo::LaserRangeFinder& lrf, const std::vector<geo::Transform2>& particles,
                        const geo::Transform2& real_pos, const WorldModel2D& wm, int i_particle = -1)
{
    Model2D particle_model = createParticle();

    cv::Scalar color1(100, 100, 255);
    cv::Scalar color2(0, 0, 255);

    for(unsigned int i = 0; i < particles.size(); ++i)
    {
        const geo::Transform2& p = particles[i];
        drawModel(canvas, particle_model, p, color1, 1);
    }

    if (i_particle >= 0)
    {
        const geo::Transform2& p = particles[i_particle];

        drawModel(canvas, particle_model, p, color2, 2);

        Canvas sub_canvas = canvas.createSubCanvas(0.1, 0.1, 0.3, 0.3);
        sub_canvas.center.y = 0.8 * sub_canvas.height();
        sub_canvas.pixels_per_meter = canvas.pixels_per_meter / 1.5;

        std::vector<double> ranges_particle = renderLRF(lrf, p, wm);
        std::vector<double> ranges_measured = renderLRF(lrf, real_pos, wm);

        geo::Transform2 sub_pose(geo::Mat2(0, 1, -1, 0), geo::Vec2(0, 0));
        cv::Point sub_pose_cv = sub_canvas.worldToImage(sub_pose.t);

        for(unsigned int i = 0; i < ranges_particle.size(); ++i)
        {
            double rp = ranges_particle[i];
            double rm = ranges_measured[i];

            const geo::Vec3& ray_dir = lrf.getRayDirection(i);

            if (rp > 0 && rm > 0)
            {
                cv::Point pp_cv = sub_canvas.worldToImage(sub_pose * geo::Vec2(rp * ray_dir.x, rp * ray_dir.y));
                cv::Point pm_cv = sub_canvas.worldToImage(sub_pose * geo::Vec2(rm * ray_dir.x, rm * ray_dir.y));

                cv::line(sub_canvas.image, sub_pose_cv, pp_cv, cv::Scalar(230, 230, 230));
                cv::line(sub_canvas.image, sub_pose_cv, pm_cv, cv::Scalar(230, 230, 230));
                cv::line(sub_canvas.image, pp_cv, pm_cv, cv::Scalar(100, 100, 100), 1);

                cv::circle(sub_canvas.image, pp_cv, 3, cv::Scalar(0, 0, 255), CV_FILLED);
                cv::circle(sub_canvas.image, pm_cv, 3, cv::Scalar(0, 150, 0), CV_FILLED);
            }
        }

        drawModel(sub_canvas, particle_model, sub_pose, color2, 2);
    }

    drawModel(canvas, particle_model, real_pos, cv::Scalar(0, 255, 0), 2);



}

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    Canvas canvas(800, 600, cv::Scalar(255, 255, 255));

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    WorldModel2D wm;
    wm.addEntity(createBox(geo::Vec2(-4, -2), geo::Vec2(4, 2), true), geo::Transform2::identity());
    wm.addEntity(createBox(geo::Vec2(1.5, -1.5), geo::Vec2(2, -2)), geo::Transform2::identity());
    wm.addEntity(createCircle(0.5), geo::Transform2(geo::Mat2::identity(), geo::Vec2(-1.5, -1.5)));

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    geo::LaserRangeFinder lrf;
    lrf.setNumBeams(20);
    lrf.setAngleLimits(-1.2, 1.2);
    lrf.setRangeLimits(0, 10);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    {
        std::vector<geo::Transform2> particles;
        for(double y = -1; y < 1.5; y += 0.5)
            for(double x = -3; x < 3.5; x += 0.5)
                for(double a = 0; a < 6; a += M_PI / 4)
                    particles.push_back(fromXYA(x, y, a));

        geo::Transform2 real_pos;
        real_pos.t = geo::Vec2(2.5, -0.5);
        real_pos.setRotation(-M_PI / 4);

        for(int i = 199; i < particles.size(); ++i)
        {

            std::cout << i << std::endl;

            canvas.clear();

            drawWorld(canvas, wm);
            drawParticleFilter(canvas, lrf, particles, real_pos, wm, i);

            std::cout << particles[i] << std::endl;

            cv::imshow("image", canvas.image);
            cv::waitKey();
        }

        return 0;
    }



    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -



//    geo::Transform2 lrf_pose = geo::Transform2::identity();

//    double a = 0;
//    while(true)
//    {
//        a += 0.1;

//        lrf_pose.setRotation(a);
//        lrf_pose.t.x = cos(a);

//        canvas.clear();

//        drawWorld(canvas, wm);
//        renderLRF(canvas, lrf, lrf_pose, wm);

//        cv::imshow("image", canvas.image);
//        if ((char)cv::waitKey(30) == 'q')
//            break;
//    }

    return 0;
}
