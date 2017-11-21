#include "rviz_draw.h"

RViz_Draw::RViz_Draw(ros::Publisher mp)
{
    marker_pub = mp;
    points.header.frame_id ="/map";
    points.header.stamp = ros::Time::now();
    points.ns = "lab3";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.z = -0.7071; //to match amcl map
    points.pose.orientation.w = 0.7071;
    points.pose.position.x = -1;
    points.pose.position.y = 5;
    points.id = 0;

    //points formatting
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.color.r = 1.0;
    points.color.a = 1.0;
}

void RViz_Draw::add_point(double x, double y)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    points.points.push_back(p);
}

void RViz_Draw::update_scale(double scalex, double scaley)
{
    points.scale.x = scalex;
    points.scale.y = scaley;
}

void RViz_Draw::update_color(double colorr, double colorg, double colorb, double colora)
{
    points.color.r = colorr;
    points.color.g = colorg;
    points.color.b = colorb;
    points.color.a = colora;
}

void RViz_Draw::pub()
{
    marker_pub.publish(points);
}
