#ifndef RVIZ_DRAW_H
#define RVIZ_DRAW_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

class RViz_Draw
{
public:
    RViz_Draw(ros::Publisher marker_pub);
    void add_point(double x, double y);
    void update_scale(double scalex, double scaley);
    void update_color(double colorr, double colorg, double colorb, double colora);
    void pub();

private:
    visualization_msgs::Marker points;
    ros::Publisher marker_pub;


};

#endif
