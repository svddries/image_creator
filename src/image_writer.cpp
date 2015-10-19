#include "image_writer.h"

#include <opencv2/highgui/highgui.hpp>
#include <iomanip>

// ----------------------------------------------------------------------------------------------------

ImageWriter::ImageWriter(int width, int height, const cv::Scalar& background_color) : canvas_(width, height, background_color),
    do_show_(true), do_write_(false)
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
    if (do_show_)
    {
        cv::imshow("image", canvas.image);
        char key = cv::waitKey();
        if (key == 'q')
            exit(0);
    }

    if (do_write_)
    {
        if (path_.empty())
        {
            std::cout << "Write path not set!" << std::endl;
            return;
        }

        if (label_.empty())
        {
            std::cout << "Label not set!" << std::endl;
            return;
        }


        std::stringstream s_filename;
//        s_filename << path_ << "/" << label_ << "-" << std::setfill('0') << std::setw(4) << image_num_ << ".png";
        s_filename << path_ << "/" << label_ << "-" << image_num_ << ".png";

        std::cout << s_filename.str() << std::endl;

        cv::imwrite(s_filename.str(), canvas.image);

        ++image_num_;
    }
}

