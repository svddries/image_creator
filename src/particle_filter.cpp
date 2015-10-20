#include "particle_filter.h"
#include "lrf.h"

#include <opencv2/imgproc/imgproc.hpp>

// ----------------------------------------------------------------------------------------------------

double prob(double r_meas, double r_hyp)
{
    // DEFAULT:
    double z_hit = 0.95;
    double sigma_hit = 0.2;
    double z_short = 0.5;
    double z_max = 0.05;
    double z_rand = 0.05;
    double lambda_short = 1;
    double range_max = 10;      // m

    double diff = r_meas - r_hyp;
    double p_hit = std::exp(-(diff * diff) / (2 * sigma_hit * sigma_hit));
    double p_short = z_short * lambda_short * std::exp(-lambda_short * r_meas);

    double p_rand = 0.03 + p_hit;

    double p = p_rand;
    if (r_meas < r_hyp)
        p += p_short;

    return p;
}

// ----------------------------------------------------------------------------------------------------

void drawLine(Canvas& canvas, const geo::Vec2& p1, const geo::Vec2& p2, const Color& color)
{
    cv::Point p1_img = canvas.worldToImage(p1);
    cv::Point p2_img = canvas.worldToImage(p2);
    cv::line(canvas.image, p1_img, p2_img, color.color, color.thickness, CV_AA);
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

void drawGraph(Canvas& graph_canvas)
{
    drawLine(graph_canvas, geo::Vec2(-10, 0), geo::Vec2(10, 0), Color(100, 100, 100, 2));
    drawLine(graph_canvas, geo::Vec2(0, -10), geo::Vec2(0, 10), Color(100, 100, 100, 2));

    for(double r_meas = 0; r_meas < 4; r_meas += 0.01)
    {
        double r_meas_old = r_meas - 0.01;
        double r_hyp = 2;

        geo::Vec2 p1(r_meas_old - r_hyp, -prob(r_meas_old, r_hyp) * 1.5);
        geo::Vec2 p2(r_meas - r_hyp, -prob(r_meas, r_hyp) * 1.5);

        drawLine(graph_canvas, p1, p2, Color(0, 0, 255, 2));
    }
}

// ----------------------------------------------------------------------------------------------------

std::vector<geo::Transform2> filterParticles(const geo::LaserRangeFinder& lrf, const std::vector<geo::Transform2>& particles,
                     const std::vector<double>& ranges_real, const WorldModel2D& wm)
{
    std::vector<geo::Transform2> new_particles;

    double total_prob = 0;

    std::vector<double> particle_probs(particles.size(), 1);
    for(unsigned int i = 0; i < particles.size(); ++i)
    {
        const geo::Transform2& p = particles[i];
        std::vector<double> ranges_hyp = renderLRF(lrf, p, wm);

        for(unsigned j = 0; j < ranges_hyp.size(); ++j)
            particle_probs[i] *= prob(ranges_real[j], ranges_hyp[j]);

        total_prob += particle_probs[i];
    }

    // normalize
    for(unsigned int i = 0; i < particles.size(); ++i)
    {
        particle_probs[i] *= (1.0 / total_prob);
        if (particle_probs[i] > 0.00001)
            new_particles.push_back(particles[i]);
    }

    return new_particles;
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

    Canvas test_canvas = canvas.createSubCanvas(0.1, 0.2, 0.35, 0.35);
    test_canvas.center.y = 0.9 * test_canvas.height();
    test_canvas.pixels_per_meter = canvas.pixels_per_meter / 1.5;

    geo::Transform2 test_pose = fromXYADegrees(0, 0, -90);

    drawRanges(test_canvas, lrf, test_pose, ranges_real, Color(0, 150, 0, 3), Color(150, 150, 150, 1));
    drawParticle(test_canvas, test_pose, Color(0, 150, 0, 2));

    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Color particle_color(255, 100, 100, 1);
    Color particle_color_bold(255, 0, 0, 2);

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

    drawParticle(canvas, particles[0], particle_color);
    iw.process(canvas);

    drawParticle(canvas, particles[8], particle_color);
    iw.process(canvas);

    drawParticle(canvas, particles[16], particle_color);
    iw.process(canvas);

    for(int i = 0; i < particles.size(); i += 8)
        drawParticle(canvas, particles[i], particle_color);
    iw.process(canvas);

    for(int i = 0; i < particles.size(); i += 8)
        drawParticle(canvas, particles[i + 1], particle_color);
    drawParticle(canvas, real_pose, Color(0, 150, 0, 2));
    iw.process(canvas);

    for(int i = 0; i < particles.size(); i += 8)
        drawParticle(canvas, particles[i + 2], particle_color);
    drawParticle(canvas, real_pose, Color(0, 150, 0, 2));
    iw.process(canvas);

    for(int i = 0; i < particles.size(); i += 1)
        drawParticle(canvas, particles[i], particle_color);
    drawParticle(canvas, real_pose, Color(0, 150, 0, 2));
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    const geo::Transform2& particle = particles[0];
    cv::Mat img_particles = canvas.image.clone();
    drawParticle(canvas, particle, particle_color_bold);
    iw.process(canvas);

    std::vector<double> ranges_hyp = renderLRF(lrf, particle, wm);
    drawRanges(canvas, lrf, particle, ranges_hyp, Color(255, 0, 0, 3), Color(150, 150, 150, 1));
    drawParticle(canvas, particle, particle_color_bold);

    iw.process(canvas);

    drawRanges(test_canvas, lrf, test_pose, ranges_hyp, Color(255, 0, 0, 3), Color(150, 150, 150, 1));
    drawParticle(test_canvas, test_pose, particle_color_bold);

    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Show graph

    cv::Mat temp = canvas.image.clone();

    Canvas graph_canvas = canvas.createSubCanvas(0.525, 0.2, 0.35, 0.35);
    graph_canvas.center.y = 0.9 * graph_canvas.height();
    graph_canvas.pixels_per_meter = canvas.pixels_per_meter;
    drawGraph(graph_canvas);

    iw.process(canvas);

    // TODO: show association process

    canvas.image = temp.clone();
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    canvas.image = img_particles.clone();
    iw.process(canvas);

    int num_i_examples = 10;
    int i_examples[] = {1, 2, 3, 8, 9, 16, 103, 209, 291, 305};

    for(int k = 0; k < num_i_examples; ++k)
    {
        int i = i_examples[k];

        canvas.image = img_particles.clone();
        Canvas test_canvas = canvas.createSubCanvas(0.1, 0.2, 0.35, 0.35);
        test_canvas.center.y = 0.9 * test_canvas.height();
        test_canvas.pixels_per_meter = canvas.pixels_per_meter / 1.5;

        drawRanges(test_canvas, lrf, test_pose, ranges_real, Color(0, 150, 0, 3), Color(150, 150, 150, 1));
        drawParticle(test_canvas, test_pose, Color(0, 150, 0, 2));

        const geo::Transform2& particle = particles[i];
        drawParticle(canvas, particle, particle_color_bold);

        std::vector<double> ranges_hyp = renderLRF(lrf, particle, wm);
        drawRanges(canvas, lrf, particle, ranges_hyp, Color(255, 0, 0, 3), Color(150, 150, 150, 1));
        drawParticle(canvas, particle, particle_color_bold);
        drawRanges(test_canvas, lrf, test_pose, ranges_hyp, Color(255, 0, 0, 3), Color(150, 150, 150, 1));
        drawParticle(test_canvas, test_pose, particle_color_bold);

        iw.process(canvas);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    canvas = iw.nextCanvas();

    for(int i = 0; i < particles.size(); ++i)
        drawParticle(canvas, particles[i], particle_color);

    particles = filterParticles(lrf, particles, ranges_real, wm);

    for(int i = 0; i < particles.size(); ++i)
        drawParticle(canvas, particles[i], particle_color_bold);

    drawWorld(canvas, wm);

    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    geo::Transform2 odom = fromXYADegrees(0, 0, 45);

    for(int i = 0; i < 3; ++i)
    {
        if (i > 0)
            real_pose = real_pose * odom;

        canvas = iw.nextCanvas();
        drawWorld(canvas, wm);

        for(int j = 0; j < particles.size(); ++j)
            drawParticle(canvas, particles[j], particle_color);

        ranges_real = renderLRF(lrf, real_pose, wm);
        drawRanges(canvas, lrf, real_pose, ranges_real, Color(0, 150, 0, 3), Color(150, 150, 150, 1));
        drawParticle(canvas, real_pose, Color(0, 150, 0, 2));
        iw.process(canvas);

    }

    test_canvas = canvas.createSubCanvas(0.1, 0.2, 0.35, 0.35);
    test_canvas.center.y = 0.9 * test_canvas.height();
    test_canvas.pixels_per_meter = canvas.pixels_per_meter / 1.5;

    drawRanges(test_canvas, lrf, test_pose, ranges_real, Color(0, 150, 0, 3), Color(150, 150, 150, 1));
    drawParticle(test_canvas, test_pose, Color(0, 150, 0, 2));
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    for(int i = 0; i < 3; ++i)
    {
        canvas = iw.nextCanvas();
        drawWorld(canvas, wm);

        test_canvas = canvas.createSubCanvas(0.1, 0.2, 0.35, 0.35);
        test_canvas.center.y = 0.9 * test_canvas.height();
        test_canvas.pixels_per_meter = canvas.pixels_per_meter / 1.5;

        drawParticle(canvas, real_pose, Color(0, 150, 0, 2));

        drawRanges(test_canvas, lrf, test_pose, ranges_real, Color(0, 150, 0, 3), Color(150, 150, 150, 1));
        drawParticle(test_canvas, test_pose, Color(0, 150, 0, 2));

        for(int j = 0; j < particles.size(); ++j)
        {
            if (i > 0)
                particles[j] =  particles[j] * odom;

            drawParticle(canvas, particles[j], particle_color);
        }

        iw.process(canvas);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    for(int k = 0; k < particles.size() + 1; ++k)
    {
        int i = k;
        if (k == particles.size())
        {
            i = 1;
        }

        canvas = iw.nextCanvas();
        drawWorld(canvas, wm);

        if (k < particles.size())
        {
            drawParticle(canvas, real_pose, Color(0, 150, 0, 2));
            for(int j = 0; j < particles.size(); ++j)
                drawParticle(canvas, particles[j], particle_color);
        }

        Canvas test_canvas = canvas.createSubCanvas(0.1, 0.2, 0.35, 0.35);
        test_canvas.center.y = 0.9 * test_canvas.height();
        test_canvas.pixels_per_meter = canvas.pixels_per_meter / 1.5;

        drawRanges(test_canvas, lrf, test_pose, ranges_real, Color(0, 150, 0, 3), Color(150, 150, 150, 1));
        drawParticle(test_canvas, test_pose, Color(0, 150, 0, 2));

        const geo::Transform2& particle = particles[i];
        drawParticle(canvas, particle, particle_color_bold);

        std::vector<double> ranges_hyp = renderLRF(lrf, particle, wm);
        drawRanges(canvas, lrf, particle, ranges_hyp, Color(255, 0, 0, 3), Color(150, 150, 150, 1));
        drawParticle(canvas, particle, particle_color_bold);
        drawRanges(test_canvas, lrf, test_pose, ranges_hyp, Color(255, 0, 0, 3), Color(150, 150, 150, 1));
        drawParticle(test_canvas, test_pose, particle_color_bold);

        iw.process(canvas);
    }







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


