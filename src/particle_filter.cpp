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

void drawParticle(Canvas& canvas, const geo::Transform2& particle, const Color& color)
{
    Model2D particle_model = createParticle();
    drawModel(canvas, particle_model, particle, color);
}

// ----------------------------------------------------------------------------------------------------

void drawParticles(Canvas& canvas, const std::vector<geo::Transform2>& particles, const Color& color)
{
    Model2D particle_model = createParticle();

    for(unsigned int i = 0; i < particles.size(); ++i)
    {
        const geo::Transform2& p = particles[i];
        drawModel(canvas, particle_model, p, color);
    }
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

void particleFilterSection(ImageWriter& iw)
{
    iw.setLabel("pf");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    WorldModel2D wm;

    Model2D room;
    {
        Contour2D& c = room.addContour();
        c.addPoint(-2, -2);
        c.addPoint( 2, -2);

        c.addPoint( 2,  1);
        c.addPoint( 1,  1);
        c.addPoint( 1,  2);

        c.addPoint(-2,  2);
    }

    geo::Transform2 room_offset = fromXYADegrees(2, 0, 0);

    wm.addEntity(room, room_offset);
//    wm.addEntity(createBox(geo::Vec2(1.5, -1.5), geo::Vec2(2, -2)), geo::Transform2::identity());
//    wm.addEntity(createCircle(0.5), geo::Transform2(geo::Mat2::identity(), geo::Vec2(-1.5, -1.5)));

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    geo::LaserRangeFinder lrf;
    lrf.setNumBeams(20);
    lrf.setAngleLimits(-1.2, 1.2);
    lrf.setRangeLimits(0, 10);

    geo::Transform2 real_pose = room_offset * fromXYADegrees(1, -1, -45);
    geo::Transform2 test_pose = fromXYADegrees(-3, 1.425, -90);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Canvas canvas = iw.nextCanvas();
    drawWorld(canvas, wm);
    iw.process(canvas);

    drawParticle(canvas, real_pose, Color(0, 150, 0, 2));
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<double> ranges_real = renderLRF(lrf, real_pose, wm);
    drawRanges(canvas, lrf, real_pose, ranges_real, Color(0, 150, 0, 3), Color(150, 150, 150, 1));
    drawParticle(canvas, real_pose, Color(0, 150, 0, 2));

    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    drawRanges(canvas, lrf, test_pose, ranges_real, Color(0, 150, 0, 3), Color(150, 150, 150, 1));
    drawParticle(canvas, test_pose, Color(0, 150, 0, 2));

    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    std::vector<geo::Transform2> particles;
    for(double y = -1.5; y < 2; y += 0.5)
    {
        for(double x = -1.5; x < 2; x += 0.5)
        {
            for(double a = 0; a < 6; a += M_PI / 4)
            {
                if (x < 1 || y < 1)
                    particles.push_back(room_offset * fromXYA(x, y, a));
            }
        }
    }

    drawParticles(canvas, particles, Color(255, 100, 100, 1));
    iw.process(canvas);










//    geo::Transform2 real_pos;
//    real_pos.t = geo::Vec2(2.5, -0.5);
//    real_pos.setRotation(-M_PI / 4);

//    for(int i = 199; i < particles.size(); ++i)
//    {

////        std::cout << i << std::endl;

//        Canvas canvas = iw.nextCanvas();

//        drawWorld(canvas, wm);
//        drawParticleFilter(canvas, lrf, particles, real_pos, wm, i);

////        std::cout << particles[i] << std::endl;

//        iw.process(canvas);
//    }
}


