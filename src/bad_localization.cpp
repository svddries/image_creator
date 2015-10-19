#include "bad_localization.h"

#include "image_writer.h"
#include "world_model.h"
#include "lrf.h"

// ----------------------------------------------------------------------------------------------------

void badLocalization(ImageWriter& iw)
{
    iw.setLabel("bad-localization");

    WorldModel2D wm;
    wm.addEntity(createBox(geo::Vec2(-3, -1.99), geo::Vec2(3, 1.99), true), geo::Transform2::identity(), Color(0, 0, 255, 2));

    WorldModel2D wm_real;
    wm_real.addEntity(createBox(geo::Vec2(-4, -2), geo::Vec2(4, 2), true), geo::Transform2::identity());

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    geo::LaserRangeFinder lrf;
    lrf.setNumBeams(50);
    lrf.setAngleLimits(-1.2, 1.2);
    lrf.setRangeLimits(0, 10);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Draw world model without extra object

    Canvas canvas = iw.nextCanvas();
    drawWorld(canvas, wm_real);
    iw.process(canvas);

    drawWorld(canvas, wm);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    for(int i = 0; i < 4; ++i)
    {
        geo::Transform2 offset = geo::Transform2::identity();
        if (i < 3)
        {
            offset.t.x = ((i + 1) / 2) * 1.0;
            if (i % 2 != 0)
                offset.t.x = -offset.t.x;
        }

        Canvas canvas = iw.nextCanvas();
        drawWorld(canvas, wm_real);

        WorldModel2D wm_t = wm.createTransformed(offset);
        drawWorld(canvas, wm_t);

        geo::Transform2 lrf_pose = fromXYADegrees(0, 1.7, -90);
        std::vector<double> ranges_virtual = renderLRF(lrf, offset * lrf_pose, wm_t);
        std::vector<double> ranges_real = renderLRF(lrf, offset * lrf_pose, wm_real);
        drawRanges(canvas, lrf, offset * lrf_pose, ranges_virtual, Color(255, 0, 0, 3));
        drawRanges(canvas, lrf, offset * lrf_pose, ranges_real, Color(0, 150, 0, 3), Color(200, 200, 200));
        iw.process(canvas);
    }
}
