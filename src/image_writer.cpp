#include "image_writer.h"

#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

ImageWriter::ImageWriter(int width, int height, const cv::Scalar& background_color) : canvas_(width, height, background_color)
{
}

// ----------------------------------------------------------------------------------------------------

ImageWriter::~ImageWriter()
{
}

// ----------------------------------------------------------------------------------------------------

Canvas ImageWriter::nextCanvas()
{
    canvas_.clear();
    return canvas_;
}

// ----------------------------------------------------------------------------------------------------

void ImageWriter::process(const Canvas& canvas)
{
    cv::imshow("image", canvas.image);
    char key = cv::waitKey();
    if (key == 'q')
        exit(0);
}

