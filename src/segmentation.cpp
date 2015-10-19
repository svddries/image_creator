#include "segmentation.h"

#include "image_writer.h"
#include "world_model.h"
#include "lrf.h"

// ----------------------------------------------------------------------------------------------------

cv::Mat createWatermark(const cv::Mat& image, const cv::Vec3b& blend_color, double blend_alpha)
{
    cv::Mat watermark(image.rows, image.cols, CV_8UC3);
    for(unsigned int i = 0; i < image.cols * image.rows; ++i)
    {
        const cv::Vec3b& oc = image.at<cv::Vec3b>(i);
        watermark.at<cv::Vec3b>(i) = blend_alpha * blend_color + (1 - blend_alpha) * oc;
    }

    return watermark;
}

// ----------------------------------------------------------------------------------------------------

void segmentationSection(ImageWriter& iw)
{
    iw.setLabel("segmentation");

    WorldModel2D wm;
    wm.addEntity(createBox(geo::Vec2(-4, -2), geo::Vec2(4, 2), true), geo::Transform2::identity());
//    wm.addEntity(createBox(geo::Vec2(1.5, -1.5), geo::Vec2(2, -2)), geo::Transform2::identity());
//    wm.addEntity(createCircle(0.5), geo::Transform2(geo::Mat2::identity(), geo::Vec2(-1.5, -1.5)));

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    geo::LaserRangeFinder lrf;
    lrf.setNumBeams(50);
    lrf.setAngleLimits(-1.2, 1.2);
    lrf.setRangeLimits(0, 10);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Draw world model without extra object

    Canvas canvas = iw.nextCanvas();
    drawWorld(canvas, wm);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Render world model (LRF) without extra object (virtual data)

    geo::Transform2 lrf_pose = fromXYADegrees(0, 1, -90);
    std::vector<double> ranges_virtual = renderLRF(lrf, lrf_pose, wm);
    drawRanges(canvas, lrf, lrf_pose, ranges_virtual, Color(255, 0, 0, 3), Color(200, 200, 200));
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Draw world model with extra object

    WorldModel2D wm_real = wm;
    wm_real.addEntity(createBox(0.8, 0.8), fromXYA(1.5, -1.5, 0), Color(0, 0, 255, 2));

    canvas = iw.nextCanvas();
    drawWorld(canvas, wm_real);
    drawRanges(canvas, lrf, lrf_pose, ranges_virtual, Color(255, 0, 0, 3), Color(200, 200, 200));
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Render world model (LRF) with extra object (real data)

    geo::Transform2 lrf_pose_real = lrf_pose;
    lrf_pose_real.t.y += 0.03;

    canvas = iw.nextCanvas();
    drawWorld(canvas, wm_real);
    std::vector<double> ranges_real = renderLRF(lrf, lrf_pose, wm_real);
    drawRanges(canvas, lrf, lrf_pose, ranges_virtual, Color(255, 0, 0, 3));
    drawRanges(canvas, lrf, lrf_pose_real, ranges_real, Color(0, 150, 0, 3), Color(200, 200, 200));
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Show association

    cv::Mat watermark = createWatermark(canvas.image, cv::Vec3b(255, 255, 255), 0.5);

    std::vector<cv::Point> points_virtual;
    std::vector<cv::Point> points_real;
    rangesToImagePoints(canvas, lrf, lrf_pose, ranges_virtual, points_virtual);
    rangesToImagePoints(canvas, lrf, lrf_pose_real, ranges_real, points_real);

    std::vector<cv::Point> non_associated;

    for(unsigned int i = 0; i < ranges_real.size(); ++i)
    {
        canvas = iw.nextCanvas();
        canvas.image = watermark.clone();

        const cv::Point& p1 = points_virtual[i];
        const cv::Point& p2 = points_real[i];

        cv::Point diff = p1 - p2;

        double dist = sqrt(diff.x * diff.x + diff.y * diff.y);
        if (dist < 8 && i > 5 && i % 5 != 0)
            continue;

        double length = std::max<int>(10, dist / 2 + 15);
        double a = lrf.getAngles()[i];

        cv::ellipse(canvas.image, 0.5 * (p1 + p2), cv::Size(10, length), a / M_PI * 180, 0, 360, cv::Scalar(0, 0, 255), 2);

        if (dist > 8)
        {
            // no association
            non_associated.push_back(p2);
        }

        for(std::vector<cv::Point>::const_iterator it = non_associated.begin(); it != non_associated.end(); ++it)
            cv::circle(canvas.image, *it, 5, cv::Scalar(255, 0, 0), 2);

        iw.process(canvas);
    }

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    canvas = iw.nextCanvas();
    drawWorld(canvas, wm);

    for(std::vector<cv::Point>::const_iterator it = non_associated.begin(); it != non_associated.end(); ++it)
        cv::circle(canvas.image, *it, 5, cv::Scalar(255, 0, 0), 2);

    iw.process(canvas);

    Model2D new_model;
    Contour2D& new_contour = new_model.addContour();

    for(unsigned int i = 0; i < non_associated.size(); ++i)
    {
        const cv::Point& p1_image = non_associated[i];
        geo::Vec2 p1_world = canvas.imageToWorld(p1_image);

        new_contour.addPoint(p1_world.x, p1_world.y);
    }

    wm.addEntity(new_model, geo::Transform2::identity());
    drawWorld(canvas, wm);

    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    canvas = iw.nextCanvas();
    drawWorld(canvas, wm);
    ranges_virtual = renderLRF(lrf, lrf_pose, wm);
    ranges_real = renderLRF(lrf, lrf_pose, wm_real);
    drawRanges(canvas, lrf, lrf_pose, ranges_virtual, Color(255, 0, 0, 3));
    drawRanges(canvas, lrf, lrf_pose_real, ranges_real, Color(0, 150, 0, 3), Color(200, 200, 200));
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    lrf_pose = fromXYADegrees(3.5, 0, -135);
    lrf_pose_real = lrf_pose;
    lrf_pose_real.t.y += 0.03;

    canvas = iw.nextCanvas();
    drawWorld(canvas, wm);
    ranges_virtual = renderLRF(lrf, lrf_pose, wm);
    ranges_real = renderLRF(lrf, lrf_pose, wm_real);
    drawRanges(canvas, lrf, lrf_pose, ranges_virtual, Color(255, 0, 0, 3));
    drawRanges(canvas, lrf, lrf_pose_real, ranges_real, Color(0, 150, 0, 3), Color(200, 200, 200));
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    rangesToImagePoints(canvas, lrf, lrf_pose, ranges_virtual, points_virtual);
    rangesToImagePoints(canvas, lrf, lrf_pose_real, ranges_real, points_real);

    for(unsigned int i = 0; i < ranges_real.size(); ++i)
    {
        double rv = ranges_virtual[i];
        double rr = ranges_real[i];

        if (std::abs(rv - rr) > 0.1)
            cv::circle(canvas.image, points_real[i], 5, cv::Scalar(255, 0, 0), 2);
    }

    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    wm.entities.pop_back();
    wm.addEntity(createBox(0.8, 0.8), fromXYA(1.5, -1.5, 0), Color(0, 0, 0, 2));

    canvas = iw.nextCanvas();
    drawWorld(canvas, wm);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    ranges_virtual = renderLRF(lrf, lrf_pose, wm);
    ranges_real = renderLRF(lrf, lrf_pose, wm_real);
    drawRanges(canvas, lrf, lrf_pose, ranges_virtual, Color(255, 0, 0, 3));
    drawRanges(canvas, lrf, lrf_pose_real, ranges_real, Color(0, 150, 0, 3), Color(200, 200, 200));
    iw.process(canvas);
}
