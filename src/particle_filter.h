#ifndef _PARTICLE_FILTER_H_
#define _PARTICLE_FILTER_H_

#include "canvas.h"
#include "world_model.h"
#include "image_writer.h"

#include <geolib/sensors/LaserRangeFinder.h>

void particleFilterSection(ImageWriter& iw, const geo::LaserRangeFinder& lrf, const WorldModel2D& wm);

#endif
