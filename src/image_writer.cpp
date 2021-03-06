#include "image_writer.h"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iomanip>

// ----------------------------------------------------------------------------------------------------

struct MouseUserData
{
    const Canvas* canvas;
};

void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
    MouseUserData* data = static_cast<MouseUserData*>(userdata);
    const Canvas& canvas = *data->canvas;

    if (event == cv::EVENT_LBUTTONDOWN)
    {
        std::cout << "Click: " << canvas.imageToWorld(cv::Point(x, y)) << std::endl;
    }
}

// ----------------------------------------------------------------------------------------------------

ImageWriter::ImageWriter(int width, int height, const geo::Vec2& p1, const geo::Vec2& p2, const cv::Scalar& background_color)
    : canvas_(width, height, background_color), do_show_(true), do_write_(false)
{
    canvas_.pixels_per_meter = width / (p2.x - p1.x);
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
        MouseUserData data;
        data.canvas = &canvas;

        cv::imshow("image", canvas.image);
        cv::setMouseCallback("image", CallBackFunc, &data);
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

// ----------------------------------------------------------------------------------------------------

void drawImage(Canvas& canvas, const std::string& filename, double width)
{
    cv::Mat image = cv::imread(filename);

    int width_pixels = canvas.width() * width;
    double f = (double)width_pixels / image.cols;
    int height_pixels = f * image.rows;

    cv::Mat image_resized;
    cv::resize(image, image_resized, cv::Size(width_pixels, height_pixels));

    cv::Mat roi = canvas.image(cv::Rect((canvas.image.cols - width_pixels) / 2,
                                        (canvas.image.rows - height_pixels) / 2,
                                        width_pixels, height_pixels));

    image_resized.copyTo(roi);
}
