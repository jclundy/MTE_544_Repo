#ifndef RVIZ_DRAW_H
#define RVIZ_DRAW_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

class RViz_Draw
{
public:
    RViz_Draw();
    RViz_Draw(ros::NodeHandle n);
    void add_point(double x, double y);
    void update_scale(double scalex, double scaley);
    void update_color(double colorr, double colorg, double colorb, double colora);

    void claim(int type);
    void release();
    void pub();

private:
    visualization_msgs::Marker objs;
    ros::Publisher marker_pub;
    bool claimed;

};

#endif
