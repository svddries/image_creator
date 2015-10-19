#include "particle_filter.h"
#include "lrf.h"

#include <opencv2/imgproc/imgproc.hpp>

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

void drawParticleFilter(Canvas& canvas, const geo::LaserRangeFinder& lrf, const std::vector<geo::Transform2>& particles,
                        const geo::Transform2& real_pos, const WorldModel2D& wm, int i_particle = -1)
{
    Model2D particle_model = createParticle();

    Color color1(255, 100, 100, 1);
    Color color2(255, 0, 0, 2);

    for(unsigned int i = 0; i < particles.size(); ++i)
    {
        const geo::Transform2& p = particles[i];
        drawModel(canvas, particle_model, p, color1);
    }

    if (i_particle >= 0)
    {
        const geo::Transform2& p = particles[i_particle];

        drawModel(canvas, particle_model, p, color2);

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

        drawModel(sub_canvas, particle_model, sub_pose, color2);
    }

    drawModel(canvas, particle_model, real_pos, Color(0, 255, 0, 2));
}

// ----------------------------------------------------------------------------------------------------

void particleFilterSection(ImageWriter& iw, const geo::LaserRangeFinder& lrf, const WorldModel2D& wm)
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

//        std::cout << i << std::endl;

        Canvas canvas = iw.nextCanvas();

        drawWorld(canvas, wm);
        drawParticleFilter(canvas, lrf, particles, real_pos, wm, i);

//        std::cout << particles[i] << std::endl;

        iw.process(canvas);
    }
}


