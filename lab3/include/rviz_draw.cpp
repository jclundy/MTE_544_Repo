#include "rviz_draw.h"

RViz_Draw::RViz_Draw()
{
    claimed = true;
}

RViz_Draw::RViz_Draw(ros::NodeHandle n, std::string marker_topic, bool latch)
{
    marker_pub = n.advertise<visualization_msgs::Marker>(marker_topic, 1, latch);
    objs.header.frame_id ="/map";
    objs.header.stamp = ros::Time::now();
    objs.ns = "lab3";
    objs.action = visualization_msgs::Marker::ADD;
    objs.pose.orientation.z = 1;
    objs.pose.position.x = 10;
    objs.pose.position.y = 0;
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
   objs.pose.position.x = originx+10;
   objs.pose.position.y = originy;
   resolution = res;
}

//returns the point index of the new point
uint RViz_Draw::add_point(double x, double y)
{
    ROS_INFO("WARNING: volatile and unsupported");
    geometry_msgs::Point p;
    p.x = 10-x;
    p.y = y;
    p.z = 0;
    objs.points.push_back(p);
    return objs.points.size() - 1;
}

void RViz_Draw::move_point(int point_id, double x, double y)
{
    geometry_msgs::Point p;
    p.x = x;
    p.y = y;
    p.z = 0;
    objs.points[point_id] = p;
}

uint RViz_Draw::add_node(Node n)
{
    geometry_msgs::Point p;
    p.x = 10-n.xpos;
    p.y = n.ypos;
    p.z = 0;
    objs.points.push_back(p);
    return objs.points.size() - 1;
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
