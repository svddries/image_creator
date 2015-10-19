#ifndef _IMAGE_WRITER_H_
#define _IMAGE_WRITER_H_

#include "canvas.h"

class ImageWriter
{

public:

    ImageWriter(int width, int height, const cv::Scalar& background_color);

    ~ImageWriter();

    void setWritePath(const std::string& path) { path_ = path; }

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

};

#endif
