#ifndef RVIZ_DRAW_H
#define RVIZ_DRAW_H

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include "Node.h"

class RViz_Draw
{
public:
    RViz_Draw();
    RViz_Draw(ros::NodeHandle n, std::string marker_topic="visualization_marker", bool latch=true);
    void update_map_details(float res, float originx, float originy);
    uint add_point(double x, double y);
    void move_point(int point_id, double x, double y);
    void add_point_scale(double x, double y);
    uint add_node(Node n);
    void update_scale(double scalex, double scaley);
    void update_color(double colorr, double colorg, double colorb, double colora);

    void claim(int type);
    void release();
    void pub();

private:
    visualization_msgs::Marker objs;
    ros::Publisher marker_pub;
    bool claimed;
    float resolution;

};

#endif
