#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include "Astar.h"
#include "MyAstar.h"

using namespace cv;
using namespace std;
MapParamNode MapParam;
Mat Maptest;
pathplanning::AstarConfig config;
pathplanning::Astar astar;
nav_msgs::OccupancyGrid OccGridMask;
bool map_flag;
bool startpoint_flag;
bool targetpoint_flag;
bool start_flag;

void World2MapGrid(MapParamNode &MapParam, Point2d &src_point, Point &dst_point)
{
    Mat P_src = Mat(Vec2d(src_point.x, src_point.y), CV_64FC1);
    Mat P_dst = MapParam.Rotation.inv() * (P_src - MapParam.Translation);

    dst_point.x = round(P_dst.at<double>(0, 0));
    dst_point.y = MapParam.height - 1 - round(P_dst.at<double>(1, 0));
}
void MapGrid2world(MapParamNode &MapParam, Point &src_point, Point2d &dst_point)
{
    Mat P_src = Mat(Vec2d(src_point.x, MapParam.height - 1 - src_point.y), CV_64FC1);

    Mat P_dst = MapParam.Rotation * P_src + MapParam.Translation;

    dst_point.x = P_dst.at<double>(0, 0);
    dst_point.y = P_dst.at<double>(1, 0);
}

void MapCallback(const nav_msgs::OccupancyGrid &msg)
{
    cout << "mapcallback" << endl;
    // Get the parameters of map
    MapParam.resolution = msg.info.resolution;
    MapParam.height = msg.info.height;
    MapParam.width = msg.info.width;
    // The origin of the MapGrid is on the bottom left corner of the map
    MapParam.x = msg.info.origin.position.x;
    MapParam.y = msg.info.origin.position.y;

    // Calculate the pose of map with respect to the world of rviz
    double roll, pitch, yaw;
    geometry_msgs::Quaternion q = msg.info.origin.orientation;
    tf::Quaternion quat(q.x, q.y, q.z, q.w); // x, y, z, w
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    double theta = yaw;
    // cout << "yaw:" << yaw << endl;
    // 从rviz上所给定的起点和终点坐标是真实世界坐标系下的位置，需要转化为地图坐标下的表示
    // MapParam.Rotation MapParam.Translation 用于该变换
    MapParam.Rotation = Mat::zeros(2, 2, CV_64FC1);
    MapParam.Rotation.at<double>(0, 0) = MapParam.resolution * cos(theta);
    MapParam.Rotation.at<double>(0, 1) = MapParam.resolution * sin(-theta);
    MapParam.Rotation.at<double>(1, 0) = MapParam.resolution * sin(theta);
    MapParam.Rotation.at<double>(1, 1) = MapParam.resolution * cos(theta);
    MapParam.Translation = Mat(Vec2d(MapParam.x, MapParam.y), CV_64FC1);

    cout << "Map:" << endl;
    cout << "MapParam.height:" << MapParam.height << endl;
    cout << "MapParam.width:" << MapParam.width << endl;

    Mat Map(MapParam.height, MapParam.width, CV_8UC1);
    Maptest = Map;
    int GridFlag;
    for (int i = 0; i < MapParam.height; i++)
    {
        for (int j = 0; j < MapParam.width; j++)
        {
            GridFlag = msg.data[i * MapParam.width + j];
            GridFlag = (GridFlag < 0) ? 100 : GridFlag; // set Unknown to 0
            Map.at<uchar>(MapParam.height - i - 1, j) = 255 - round(GridFlag * 255.0 / 100.0);
        }
    }

    Mat Mask;
    config.InflateRadius = round(0.25 / MapParam.resolution);
    // config.InflateRadius = 5;

    astar.InitAstar(Maptest, Mask, config);

    // publish mask

    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "map";
    OccGridMask.info = msg.info;
    OccGridMask.data.clear();
    for (int i = 0; i < MapParam.height; i++)
    {
        for (int j = 0; j < MapParam.width; j++)
        {
            GridFlag = Mask.at<uchar>(MapParam.height - i - 1, j) * 255;
            OccGridMask.data.push_back(GridFlag);
        }
    }

    // Set flag
    map_flag = true;
    startpoint_flag = false;
    targetpoint_flag = false;
}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    World2MapGrid(MapParam, src_point, MapParam.StartPoint);
    startpoint_flag = true;
    if (map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }
    cout << "StartPoint:" << MapParam.StartPoint << endl;
}

void TargetPointtCallback(const geometry_msgs::PoseStamped &msg)
{
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    World2MapGrid(MapParam, src_point, MapParam.TargetPoint);
    int p = Maptest.at<uchar>(MapParam.TargetPoint.x, MapParam.TargetPoint.y);
    cout << "flag:" << p << endl;
    MapGrid2world(MapParam, MapParam.TargetPoint, src_point);
    // cout << "TargetPoint world:" << src_point << endl;
    // cout << "TargetPoint:" << MapParam.TargetPoint << endl;
    // Set flag
    targetpoint_flag = true;
    if (map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }
}
void PathGrid2world(MapParamNode &MapParam, vector<Point> &PathList, nav_msgs::Path &plan_path)
{
    plan_path.header.stamp = ros::Time::now();
    plan_path.header.frame_id = "map";
    plan_path.poses.clear();
    for (int i = 0; i < PathList.size(); i++)
    {
        Point2d dst_point;
        MapGrid2world(MapParam, PathList[i], dst_point);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time::now();
        pose_stamped.header.frame_id = "map";
        pose_stamped.pose.position.x = dst_point.x;
        pose_stamped.pose.position.y = dst_point.y;
        pose_stamped.pose.position.z = 0;
        pose_stamped.pose.orientation.w = 1.0;
        plan_path.poses.push_back(pose_stamped);
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "astar");
    ros::NodeHandle n;

    geometry_msgs::PointStamped astar_step;

    // Subscribe topics
    ros::Subscriber Map_sub = n.subscribe("map", 10, MapCallback);
    ros::Subscriber StarPoint_sub = n.subscribe("initialpose", 10, StartPointCallback);
    ros::Subscriber TargetPoint_sub = n.subscribe("move_base_simple/goal", 10, TargetPointtCallback);
    // cout << "main" << endl;

    // Publisher topics
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("move_base/NavfnROS/nav_path", 10);
    ros::Publisher mask_pub = n.advertise<nav_msgs::OccupancyGrid>("mask", 1);
    ros::Rate loop_rate(20);
    nav_msgs::Path plan_path;

    vector<Point> PathList;

    while (ros::ok())
    {
        if (start_flag)
        {
            int distance = 0;
            astar.PathPlanning(MapParam.StartPoint, MapParam.TargetPoint, PathList);
            plan_path.poses.clear();
            distance = PathList.size();
            if (!PathList.empty())
            {
                PathGrid2world(MapParam, PathList, plan_path);

                cout << "Find a valid path successfully!Distance:" << distance << endl;
            }
            else
            {
                cout << "Can not find a valid path" << endl;
            }
            path_pub.publish(plan_path);
            start_flag = false;
        }

        if (map_flag)
        {
            mask_pub.publish(OccGridMask);
        }

        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
