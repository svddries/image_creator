#include "lrf_example.h"

#include "image_writer.h"
#include "world_model.h"
#include "lrf.h"

// ----------------------------------------------------------------------------------------------------

void lrfExample(ImageWriter& iw)
{
    iw.setLabel("lrf-example");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    WorldModel2D wm;
    wm.addEntity(createBox(geo::Vec2(0, -4), geo::Vec2(8, 0), true), fromXYA(-4, 2, 0)); // 0
    wm.addEntity(createBox(1.6, 0.8), fromXYA(2, -1, 0));      // 1  table
    wm.addEntity(createCircle(0.1), fromXYA(2.6, -0.8, 0));    // 2  small object
    wm.addEntity(createCircle(0.5), fromXYA(-3.2, -1.2, 0));   // 3
    wm.addEntity(createCircle(0.3), fromXYA(-2.2, -1.4, 0));   // 4
    wm.addEntity(createCircle(0.3), fromXYA(3.3, 1.0, 0));     // 5

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
    drawLRFPose(canvas, lrf_pose, Color(0, 150, 0, 2));
    iw.process(canvas);

    std::vector<double> ranges_virtual = renderLRF(lrf, lrf_pose, wm);
    drawRanges(canvas, lrf, lrf_pose, ranges_virtual, Color(255, 0, 0, 3), Color(200, 200, 200));
    drawLRFPose(canvas, lrf_pose, Color(0, 150, 0, 2));
    iw.process(canvas);
}
