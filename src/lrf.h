#ifndef _LRF_H_
#define _LRF_H_

#include "canvas.h"

#include <geolib/sensors/LaserRangeFinder.h>

struct WorldModel2D;

// ----------------------------------------------------------------------------------------------------

std::vector<double> renderLRF(const geo::LaserRangeFinder& lrf, const geo::Transform2& lrf_pose, const WorldModel2D& wm);

// ----------------------------------------------------------------------------------------------------

void rangesToImagePoints(Canvas& canvas, geo::LaserRangeFinder& lrf, const geo::Transform2& lrf_pose, const std::vector<double>& ranges,
                         std::vector<cv::Point>& points_image);

// ----------------------------------------------------------------------------------------------------

void drawRanges(Canvas& canvas, const geo::LaserRangeFinder& lrf, const geo::Transform2& lrf_pose, const std::vector<double>& ranges,
                const Color& point_color, const Color& line_color = Color());

// ----------------------------------------------------------------------------------------------------

void drawLRFPose(Canvas& canvas, const geo::Transform2& pose, const Color& color);

// ----------------------------------------------------------------------------------------------------

#endif
