#ifndef _IMAGE_WRITER_H_
#define _IMAGE_WRITER_H_

#include "canvas.h"

class ImageWriter
{

public:

    ImageWriter(int width, int height, const cv::Scalar& background_color);

    ~ImageWriter();   

    Canvas nextCanvas();

    void process(const Canvas& canvas);


private:

    Canvas canvas_;

};

#endif
