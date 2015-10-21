#ifndef _IMAGE_WRITER_H_
#define _IMAGE_WRITER_H_

#include "canvas.h"

class ImageWriter
{

public:

    ImageWriter(int width, int height, const geo::Vec2& p1, const geo::Vec2& p2, const cv::Scalar& background_color);

    ~ImageWriter();

    void setWritePath(const std::string& path) { path_ = path; }

    void setImagePath(const std::string& path) { image_path_ = path; }

    const std::string& image_path() const { return image_path_; }

    void setShow(bool b = true) { do_show_ = b; }

    void setWrite(bool b = true) { do_write_ = b; }

    void setLabel(const std::string& label)
    {
        label_ = label;
        image_num_ = 0;
    }

    Canvas nextCanvas();

    void process(const Canvas& canvas);

private:

    Canvas canvas_;

    int image_num_;

    std::string label_;

    bool do_show_;
    bool do_write_;

    std::string path_;

    std::string image_path_;

};

void drawImage(Canvas& canvas, const std::string& filename, double width);

#endif
