#include "rviz_draw.h"

RViz_Draw::RViz_Draw()
{
    claimed = true;
}

RViz_Draw::RViz_Draw(ros::NodeHandle n)
{
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
    objs.header.frame_id ="/map";
    objs.header.stamp = ros::Time::now();
    objs.ns = "lab3";
    objs.action = visualization_msgs::Marker::ADD;
    objs.pose.orientation.z = -0.7071; //to match amcl map
    objs.pose.orientation.w = 0.7071;
    objs.pose.position.x = -5;
    objs.pose.position.y = 5;
    objs.id = 0;

    //objs formatting
    objs.type = visualization_msgs::Marker::POINTS;
    objs.scale.x = 0.05;
    objs.scale.y = 0.05;
    objs.color.r = 1.0;
    objs.color.a = 1.0;

    claimed = false;
}

void RViz_Draw::update_map_details(float res, float originx, float originy)
{
   objs.pose.position.x = originx;
   objs.pose.position.y = originy+10;
   resolution = res;
}

void RViz_Draw::add_point(double x, double y)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    objs.points.push_back(p);
}

void RViz_Draw::add_point_scale(double x, double y)
{
    geometry_msgs::Point p;
    p.x = x*resolution;
    p.y = y*resolution;
    p.z = 0;
    objs.points.push_back(p);
}

void RViz_Draw::update_scale(double scalex, double scaley)
{
    objs.scale.x = scalex;
    objs.scale.y = scaley;
}

void RViz_Draw::update_color(double colorr, double colorg, double colorb, double colora)
{
    objs.color.r = colorr;
    objs.color.g = colorg;
    objs.color.b = colorb;
    objs.color.a = colora;
}

void RViz_Draw::claim(int type)
{
    while(claimed) {ros::spinOnce();}
    claimed = true;
    objs.points.clear();
    objs.type = type;
}

void RViz_Draw::release()
{
    claimed = false;
}

void RViz_Draw::pub()
{
    marker_pub.publish(objs);
    objs.id++;
}
