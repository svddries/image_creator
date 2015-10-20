#include "image_writer.h"

#include "particle_filter.h"
#include "segmentation.h"
#include "relative.h"
#include "bad_localization.h"

// ----------------------------------------------------------------------------------------------------

int main(int argc, char **argv)
{
    ImageWriter iw(800, 600, cv::Scalar(255, 255, 255));

    bool show = true;

    if (!show && argc > 1)
    {
        iw.setWritePath(argv[1]);
        iw.setWrite(true);
//        std::cout << "Going to write!" << std::endl;
    }

    iw.setShow(show);

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

//    particleFilterSection(iw, lrf, wm);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//    segmentationSection(iw);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//    badLocalization(iw);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    relativeSection(iw);

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
