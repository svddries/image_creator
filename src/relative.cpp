#include "relative.h"

#include "image_writer.h"
#include "world_model.h"

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// ----------------------------------------------------------------------------------------------------

struct Link
{
    Link(int i1_, int i2_) : i1(i1_), i2(i2_) {}
    int i1;
    int i2;
};

// ----------------------------------------------------------------------------------------------------

void drawAxis(Canvas& canvas, const geo::Transform2& t)
{
    cv::Point p0 = canvas.worldToImage(t.t);
    cv::Point px = canvas.worldToImage((t * geo::Vec2(0.2, 0)));
    cv::Point py = canvas.worldToImage((t * geo::Vec2(0, -0.2)));

    cv::line(canvas.image, p0, px, cv::Scalar(0, 0, 255), 2);
    cv::line(canvas.image, p0, py, cv::Scalar(0, 150, 0), 2);
}

// ----------------------------------------------------------------------------------------------------

void drawArrow(Canvas& canvas, const geo::Vec2& p1, const geo::Vec2& p2, const Color& color, bool dashed = false)
{
    geo::Vec2 dir = (p2 - p1).normalized();
    geo::Mat2 m(dir.x, dir.y, dir.y, -dir.x);

    geo::Vec2 phead1 = p2 + m * (geo::Vec2(-0.1, -0.07));
    geo::Vec2 phead2 = p2 + m * (geo::Vec2(-0.1,  0.07));

    cv::Point p1_img = canvas.worldToImage(p1);
    cv::Point p2_img = canvas.worldToImage(p2);
    cv::Point phead1_img = canvas.worldToImage(phead1);
    cv::Point phead2_img = canvas.worldToImage(phead2);

    if (dashed)
    {
        cv::Point2d diff = p2_img - p1_img;
        double l = sqrt(diff.x * diff.x + diff.y * diff.y);

        cv::Point2d diff_n = (1.0 / l) * diff;

        std::cout << p2_img << " " << p1_img << std::endl;
        std::cout << diff << std::endl;
        std::cout << diff_n << std::endl;
        std::cout << "   " << std::endl;

        double step = 10;
        double w = 5;

        for(double b = 0; b < l; b += step)
        {
            cv::Point2d p1a_img = cv::Point2d(p1_img.x, p1_img.y) + b * diff_n;
            cv::Point2d p2a_img = cv::Point2d(p1_img.x, p1_img.y) + (w + b) * diff_n;
            cv::line(canvas.image, p1a_img, p2a_img, color.color, color.thickness, CV_AA);
        }
    }
    else
    {
        cv::line(canvas.image, p1_img, p2_img, color.color, color.thickness, CV_AA);
    }

    cv::line(canvas.image, p2_img, phead1_img, color.color, color.thickness, CV_AA);
    cv::line(canvas.image, p2_img, phead2_img, color.color, color.thickness, CV_AA);
}

// ----------------------------------------------------------------------------------------------------

void drawWorldModelAbsolute(Canvas& canvas, const WorldModel2D& wm, const geo::Vec2& p0)
{
    drawWorld(canvas, wm);

    for(unsigned int i = 1; i < wm.entities.size(); ++i)
    {
        const Entity2D& e = wm.entities[i];
        drawArrow(canvas, p0, e.pose.t, Color(150, 150, 150, 2));
        drawAxis(canvas, e.pose);
    }

    drawAxis(canvas, fromXYA(p0.x, p0.y, 0));
}

// ----------------------------------------------------------------------------------------------------

void drawWorldModelSceneGraph(Canvas& canvas, const WorldModel2D& wm, const std::vector<Link>& links)
{
    drawWorld(canvas, wm);

    for(unsigned int i = 0; i < links.size(); ++i)
    {
        const Link& link = links[i];

        const Entity2D& e1 = wm.entities[link.i1];
        const Entity2D& e2 = wm.entities[link.i2];

        drawArrow(canvas, e1.pose.t, e2.pose.t, Color(150, 150, 150, 2));

        drawAxis(canvas, e1.pose);
        drawAxis(canvas, e2.pose);
    }

//    drawAxis(canvas, fromXYA(p0.x, p0.y, 0));
}

// ----------------------------------------------------------------------------------------------------

void drawRectangle(Canvas& canvas, const geo::Vec2& p1, const geo::Vec2& p2, const Color& color, bool filled = false)
{
    cv::Point p1_img = canvas.worldToImage(p1);
    cv::Point p2_img = canvas.worldToImage(p2);

    if (filled)
        cv::rectangle(canvas.image, p1_img, p2_img, color.color, CV_FILLED);
    else
        cv::rectangle(canvas.image, p1_img, p2_img, color.color, color.thickness);
}

// ----------------------------------------------------------------------------------------------------

void drawTriangle(Canvas& canvas, const geo::Vec2& p1, const geo::Vec2& p2, const geo::Vec2& p3, const Color& color)
{
    cv::Point p1_img = canvas.worldToImage(p1);
    cv::Point p2_img = canvas.worldToImage(p2);
    cv::Point p3_img = canvas.worldToImage(p3);

    cv::line(canvas.image, p1_img, p2_img, color.color, color.thickness);
    cv::line(canvas.image, p1_img, p3_img, color.color, color.thickness);
    cv::line(canvas.image, p2_img, p3_img, color.color, color.thickness);
}


// ----------------------------------------------------------------------------------------------------

Model2D createTarget()
{
    Model2D m;
    createCircleContour(0.02, m.addContour());
    createCircleContour(0.08, m.addContour());
    createCircleContour(0.16, m.addContour());
    createCircleContour(0.24, m.addContour());
    return m;
}

// ----------------------------------------------------------------------------------------------------

Model2D createLRFPose()
{
    Model2D model = createCircle(0.1);
    Contour2D& c = model.addContour();
    c.addPoint(0, 0);
    c.addPoint(0.2, 0);
    return model;
}

// ----------------------------------------------------------------------------------------------------

Model2D createSoccerFieldModel()
{
    Model2D m;

    geo::Vec2 p1(-4, -2.67);
    geo::Vec2 p2( 4,  2.67);

    geo::Vec2 pm1(0, p1.y);
    geo::Vec2 pm2(0, p2.y);

    createBoxContour(p1, p2, m.addContour());
    createBoxContour(pm1, pm2, m.addContour());

    createBoxContour(geo::Vec2(p1.x, -1), geo::Vec2(p1.x + 0.5, 1), m.addContour());
    createBoxContour(geo::Vec2(p2.x, -1), geo::Vec2(p2.x - 0.5, 1), m.addContour());

    createCircleContour(0.7, m.addContour());

    return m;
}

// ----------------------------------------------------------------------------------------------------

Model2D createTurtleModel()
{
    double radius = 0.3;
    double bla = sqrt(2.0) / 2 * radius;
    Model2D m = createCircle(radius);

    Contour2D& c1 = m.addContour();
    c1.addPoint(0, 0);
    c1.addPoint(bla, bla);

    Contour2D& c2 = m.addContour();
    c2.addPoint(0, 0);
    c2.addPoint(bla, -bla);

    return m;
}

// ----------------------------------------------------------------------------------------------------

void drawSoccerField(Canvas& canvas)
{
    geo::Vec2 p1(-4, -2.67);
    geo::Vec2 p2( 4,  2.67);
    geo::Vec2 border(0.2, 0.2);

    drawRectangle(canvas, p1 - border, p2 + border, Color(180, 255, 180), true);
}

// ----------------------------------------------------------------------------------------------------

void relativeSection(ImageWriter& iw)
{
    iw.setLabel("relative");

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    WorldModel2D wm;

//    wm.addEntity(createBox(geo::Vec2(), geo::Vec2()));

//    wm.addEntity(createBox(7.63, 4.09, true), fromXYADegrees(0.085, 0.065, 0));
    wm.addEntity(createBox(geo::Vec2(0, -4.09), geo::Vec2(7.63, 0), true), fromXYADegrees(-3.75, 2.1, 0));


    int idx_couch = wm.entities.size();
    wm.addEntity(createBox(0.691667, 1.45833), fromXYADegrees(-0.529166, 0.404165, 0));
    wm.addEntity(createBox(0.45833, 0.6), fromXYADegrees(-1.69583, 0.375, 0));
    wm.addEntity(createBox(0.95833, 0.516667), fromXYADegrees(-1.72917, -0.616667, 0));
    wm.addEntity(createBox(0.96666, 0.50833), fromXYADegrees(-1.75, 1.37084, 0));
    wm.addEntity(createBox(1.6, 0.30834), fromXYADegrees(-0.45, -1.8375, 0));

    int idx_table = wm.entities.size();
    wm.addEntity(createBox(0.65, 0.95), fromXYADegrees(1.6, 0.4, 0));

    int idx_cabinet = wm.entities.size();
    wm.addEntity(createBox(0.55, 1.28333), fromXYADegrees(3.54167, 0.558333, 0));

    wm.addEntity(createCircle(0.1), fromXYA(0.358333, 1.81667, 0));
    wm.addEntity(createCircle(0.1), fromXYA(-1.73333, -1.78333, 0));

    int idx_plant = wm.entities.size();
    wm.addEntity(createCircle(0.1), fromXYA(3.18333, 1.68333, 0));

    Model2D model;
    Contour2D& c = model.addContour();
    c.addPoint(-3.73333 + 3.3, -1 + 1.6);
    c.addPoint(-3.15 + 3.3, -1 + 1.6);
    c.addPoint(-3.15 + 3.3, -1.46 + 1.6);
    c.addPoint(-2.36 + 3.3, -1.46 + 1.6);
    c.addPoint(-2.36 + 3.3, -1.97 + 1.6);
    c.addPoint(-3.73333 + 3.3, -1.97 + 1.6);

    wm.addEntity(model, fromXYADegrees(-3.3, -1.6, 0));

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    Canvas canvas = iw.nextCanvas();
    drawImage(canvas, iw.image_path() + "/livingroom.jpg", 0.9);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    drawWorld(canvas, wm);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    int idx_lrf = wm.entities.size();
    wm.addEntity(createLRFPose(), fromXYADegrees(0.533333, -1.24167, 145), Color(0, 150, 0, 2));
    drawWorld(canvas, wm);
    iw.process(canvas);

    int idx_target = wm.entities.size();
    wm.addEntity(createTarget(), fromXYADegrees(-0.533333, -0.725, 0), Color(255, 0, 0, 2));
    drawWorld(canvas, wm);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Moved couch

    canvas = iw.nextCanvas();
    drawImage(canvas, iw.image_path() + "/livingroom2.jpg", 0.9);
    drawWorld(canvas, wm);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

//    canvas = iw.nextCanvas();
    drawWorldModelAbsolute(canvas, wm, wm.entities[0].pose.t);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    canvas = iw.nextCanvas();
    drawImage(canvas, iw.image_path() + "/livingroom2.jpg", 0.9);
    drawWorld(canvas, wm);
    iw.process(canvas);

    std::vector<Link> links;
    links.push_back(Link(0, idx_target));
    links.push_back(Link(0, idx_lrf));

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    drawWorldModelSceneGraph(canvas, wm, links);
    iw.process(canvas);

    canvas = iw.nextCanvas();
    drawImage(canvas, iw.image_path() + "/livingroom2.jpg", 0.9);
    drawWorld(canvas, wm);
    iw.process(canvas);

    links.clear();

    links.push_back(Link(idx_couch, idx_target));
    drawWorldModelSceneGraph(canvas, wm, links);
    iw.process(canvas);

    links.push_back(Link(idx_lrf, idx_couch));
    drawWorldModelSceneGraph(canvas, wm, links);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    WorldModel2D wm2;
    wm2.entities.push_back(wm.entities[idx_lrf]);
    wm2.entities.push_back(wm.entities[idx_couch]);
    wm2.entities.push_back(wm.entities[idx_target]);

    links.clear();
    links.push_back(Link(0, 1));
    links.push_back(Link(1, 2));

    canvas = iw.nextCanvas();
    drawWorldModelSceneGraph(canvas, wm2, links);
    iw.process(canvas);

    canvas = iw.nextCanvas();
    drawImage(canvas, iw.image_path() + "/livingroom2.jpg", 0.9);

    wm.entities[idx_target].pose.t.y += 0.7;
    wm.entities[idx_couch].pose.t.y += 0.7;

    links.clear();
    links.push_back(Link(idx_couch, idx_target));
    links.push_back(Link(idx_lrf, idx_couch));

    drawWorldModelSceneGraph(canvas, wm, links);
    iw.process(canvas);

    links.push_back(Link(idx_couch, 2));
    links.push_back(Link(2, 3));
    links.push_back(Link(2, 4));
    links.push_back(Link(idx_lrf, idx_table));
    links.push_back(Link(idx_table, idx_cabinet));
    links.push_back(Link(idx_cabinet, idx_plant));

    drawWorldModelSceneGraph(canvas, wm, links);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    // Soccer field

    iw.setLabel("relative-soccer");

//Click: [ -2.0125 -0.0125 ]
//Click: [ 0.8375 -1.1 ]
//Click: [ 2.075 0.025 ]
//Click: [ 2 1.7125 ]  ball

    wm.entities.clear();
    wm.addEntity(createSoccerFieldModel(), geo::Transform2::identity(), Color(255, 255, 255, 2));
    wm.addEntity(createCircle(0.2), fromXYA(2, 1.7125, 0), Color(255, 220, 0, 2));

    wm.addEntity(createTurtleModel(), fromXYADegrees(0.8375, -1.1 , 60), Color(0, 0, 0, 2));
//    wm.addEntity(createTurtleModel(), fromXYADegrees(-1.1, -0.15, -90), Color(0, 0, 0, 2));
//    wm.addEntity(createTurtleModel(), fromXYADegrees(2.275, 0.025, 105), Color(0, 0, 0, 2));


    canvas = iw.nextCanvas();
    drawSoccerField(canvas);
    drawWorld(canvas, wm);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    drawWorldModelAbsolute(canvas, wm, geo::Vec2(-4, 2.67));
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    links.clear();
    links.push_back(Link(2, 1));

    canvas = iw.nextCanvas();
    drawSoccerField(canvas);
    drawWorldModelSceneGraph(canvas, wm, links);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    wm.addEntity(createTurtleModel(), fromXYADegrees(-1.5, -0.15, -10), Color(0, 0, 0, 2));

    canvas = iw.nextCanvas();
    drawSoccerField(canvas);
    drawWorldModelSceneGraph(canvas, wm, links);
    iw.process(canvas);

    links.push_back(Link(3, 2));

    drawWorldModelSceneGraph(canvas, wm, links);
    iw.process(canvas);

    drawArrow(canvas, wm.entities[3].pose.t, wm.entities[1].pose.t, Color(150, 150, 150, 2), true);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    canvas = iw.nextCanvas();
    drawSoccerField(canvas);

    geo::Vec2 ball_pos_rel = wm.entities[2].pose.inverse() * wm.entities[1].pose.t;

    geo::Transform2 offset = geo::Transform2::identity();
    offset.setRotation(-0.3);

    wm.entities[2].pose = wm.entities[2].pose * offset;
    wm.entities[1].pose.t = wm.entities[2].pose * ball_pos_rel;

    drawWorldModelSceneGraph(canvas, wm, links);
    drawArrow(canvas, wm.entities[3].pose.t, wm.entities[1].pose.t, Color(150, 150, 150, 2), true);
    iw.process(canvas);

    // Turn back
    wm.entities[2].pose = wm.entities[2].pose * offset.inverse();
    wm.entities[1].pose.t = wm.entities[2].pose * ball_pos_rel;

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    canvas = iw.nextCanvas();
    drawSoccerField(canvas);
    drawWorld(canvas, wm);
    iw.process(canvas);

//    wm.addEntity(createTurtleModel(), fromXYADegrees(2.275, 0.025, 105), Color(0, 0, 0, 2));
    wm.addEntity(createTurtleModel(), fromXYADegrees(0.375, 0.65, -105), Color(0, 0, 0, 2));

    drawWorld(canvas, wm);
    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    links.clear();
    links.push_back(Link(2, 1));
    links.push_back(Link(2, 4));

    drawWorldModelSceneGraph(canvas, wm, links);
    iw.process(canvas);

//    links.push_back(Link(4, 1));

//    drawWorldModelSceneGraph(canvas, wm, links);
//    iw.process(canvas);

    links.push_back(Link(3, 2));
    links.push_back(Link(3, 4));

    drawWorldModelSceneGraph(canvas, wm, links);
    iw.process(canvas);

    cv::Mat temp = canvas.image.clone();

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    canvas = iw.nextCanvas();
    canvas.image = temp.clone();

    drawTriangle(canvas, wm.entities[1].pose.t, wm.entities[2].pose.t, wm.entities[4].pose.t, Color(0, 255, 255, 2));

    iw.process(canvas);

    // - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

    // Replace 3th turtle by field feature
    wm.entities[4] = Entity2D(Model2D(), fromXYA(0, 0.7, 0), Color());

    canvas = iw.nextCanvas();
    drawSoccerField(canvas);
    drawWorldModelSceneGraph(canvas, wm, links);
    iw.process(canvas);

    drawTriangle(canvas, wm.entities[1].pose.t, wm.entities[2].pose.t, wm.entities[4].pose.t, Color(0, 255, 255, 2));

    iw.process(canvas);


}

