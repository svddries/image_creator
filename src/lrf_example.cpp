#include "lrf_example.h"

#include "image_writer.h"
#include "world_model.h"
#include "lrf.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ----------------------------------------------------------------------------------------------------

void drawImage(Canvas& canvas, const std::string& filename, double width)
{
    cv::Mat image = cv::imread(filename);

    int width_pixels = canvas.width() * width;
    double f = (double)width_pixels / image.cols;
    int height_pixels = f * image.rows;

    std::cout << f << std::endl;

    cv::Mat image_resized;
    cv::resize(image, image_resized, cv::Size(width_pixels, height_pixels));

    cv::Mat roi = canvas.image(cv::Rect((canvas.image.cols - width_pixels) / 2,
                                        (canvas.image.rows - height_pixels) / 2,
                                        width_pixels, height_pixels));

    image_resized.copyTo(roi);
}


// ----------------------------------------------------------------------------------------------------

void lrfExample(ImageWriter& iw)
{
    iw.setLabel("lrf-example");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    WorldModel2D wm;

//    wm.addEntity(createBox(geo::Vec2(), geo::Vec2()));

    wm.addEntity(createBox(7.63, 4.09, true), fromXYADegrees(0.085, 0.065, 0));
    wm.addEntity(createBox(0.691667, 1.45833), fromXYADegrees(-0.529166, 0.404165, 0));
    wm.addEntity(createBox(0.45833, 0.6), fromXYADegrees(-1.69583, 0.375, 0));
    wm.addEntity(createBox(0.95833, 0.516667), fromXYADegrees(-1.72917, -0.616667, 0));
    wm.addEntity(createBox(0.96666, 0.50833), fromXYADegrees(-1.75, 1.37084, 0));
    wm.addEntity(createBox(1.6, 0.30834), fromXYADegrees(-0.45, -1.8375, 0));
    wm.addEntity(createBox(0.65, 0.95), fromXYADegrees(1.6, 0.4, 0));
    wm.addEntity(createBox(0.55, 1.28333), fromXYADegrees(3.54167, 0.558333, 0));

    wm.addEntity(createCircle(0.1), fromXYA(0.358333, 1.81667, 0));
    wm.addEntity(createCircle(0.1), fromXYA(-1.73333, -1.78333, 0));
    wm.addEntity(createCircle(0.1), fromXYA(3.18333, 1.68333, 0));

    Model2D model;
    Contour2D& c = model.addContour();
    c.addPoint(-3.73333 + 3.3, -1 + 1.6);
    c.addPoint(-3.15 + 3.3, -1 + 1.6);
    c.addPoint(-3.15 + 3.3, -1.46 + 1.6);
    c.addPoint(-2.36 + 3.3, -1.46 + 1.6);
    c.addPoint(-2.36 + 3.3, -1.97 + 1.6);
    c.addPoint(-3.73333 + 3.3, -1.97 + 1.6);

    wm.addEntity(model, fromXYADegrees(-3.3, -1.6, 0));


//    wm.addEntity(createBox(geo::Vec2(0, -4), geo::Vec2(8, 0), true), fromXYA(-4, 2, 0)); // 0
//    wm.addEntity(createBox(1.6, 0.8), fromXYA(2, -1, 0));      // 1  table
//    wm.addEntity(createCircle(0.1), fromXYA(2.6, -0.8, 0));    // 2  small object
//    wm.addEntity(createCircle(0.5), fromXYA(-3.2, -1.2, 0));   // 3
//    wm.addEntity(createCircle(0.3), fromXYA(-2.2, -1.4, 0));   // 4
//    wm.addEntity(createCircle(0.3), fromXYA(3.3, 1.0, 0));     // 5

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    geo::LaserRangeFinder lrf;
    lrf.setNumBeams(50);
    lrf.setAngleLimits(-1.0, 1.0);
    lrf.setRangeLimits(0, 10);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Draw world model without extra object

    Canvas canvas = iw.nextCanvas();
    drawImage(canvas, iw.image_path() + "/livingroom.jpg", 0.9);

    iw.setLabel("livingroom");
    iw.process(canvas);

    iw.setLabel("lrf-example");
    iw.process(canvas);

//    canvas = iw.nextCanvas();
//    drawWorld(canvas, wm);
//    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Render world model (LRF) without extra object (virtual data)

    geo::Transform2 lrf_pose = fromXYADegrees(0.533333, -1.24167, 145);
    drawLRFPose(canvas, lrf_pose, Color(0, 150, 0, 2));
    iw.process(canvas);

    std::vector<double> ranges_virtual = renderLRF(lrf, lrf_pose, wm);
    drawRanges(canvas, lrf, lrf_pose, ranges_virtual, Color(255, 0, 0, 3), Color(50, 50, 50));
    drawLRFPose(canvas, lrf_pose, Color(0, 150, 0, 2));
    iw.process(canvas);

    drawWorld(canvas, wm);
    drawRanges(canvas, lrf, lrf_pose, ranges_virtual, Color(255, 0, 0, 3), Color(50, 50, 50));
    drawLRFPose(canvas, lrf_pose, Color(0, 150, 0, 2));
    iw.process(canvas);

    canvas = iw.nextCanvas();
    drawWorld(canvas, wm);
    drawRanges(canvas, lrf, lrf_pose, ranges_virtual, Color(255, 0, 0, 3), Color(50, 50, 50));
    drawLRFPose(canvas, lrf_pose, Color(0, 150, 0, 2));
    iw.process(canvas);

}
