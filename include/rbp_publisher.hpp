#pragma once

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// ROS
#include <sensor_msgs/Imu.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// MATPLOTLIB-CPP
#define _USE_MATH_DEFINES
#include <cmath>

// Submodules

#include <init_traj_planner.hpp>
#include <mission.hpp>
#include <param.hpp>

class ResultPublisher {
public:
    ResultPublisher(ros::NodeHandle _nh,
//                  std::shared_ptr<RBPPlanner> _RBPPlanner_obj,
//                  std::shared_ptr<Corridor> _corridor_obj,
                  std::shared_ptr<InitTrajPlanner> _initTrajPlanner_obj,
                  MPF::Mission _mission,
                  MPF::Param _param)
            : nh(std::move(_nh)),
            //   RBPPlanner_obj(std::move(_RBPPlanner_obj)),
            //   corridor_obj(std::move(_corridor_obj)),
              initTrajPlanner_obj(std::move(_initTrajPlanner_obj)),
              mission(std::move(_mission)),
              param(std::move(_param))
    {
        qn = mission.qn;
        outdim = 3;

        initTraj_pub = nh.advertise<visualization_msgs::MarkerArray>("/initTraj", 1);

        M = initTrajPlanner_obj.get()->T.size()-1;

    }



    void update(double current_time){

        update_initTraj();

    }

    void publish(){
     
        initTraj_pub.publish(msgs_initTraj);

    }

private:
    ros::NodeHandle nh;
    // std::shared_ptr<RBPPlanner> RBPPlanner_obj;
    // std::shared_ptr<Corridor> corridor_obj;
    std::shared_ptr<InitTrajPlanner> initTrajPlanner_obj;
    MPF::Mission mission;
    MPF::Param param;

    int qn, M, outdim;
    double global_min_dist;
    tf::TransformBroadcaster br;
    std::vector<Eigen::MatrixXd> pva;
    std::vector<Eigen::MatrixXd> coef;
    std::vector<std::vector<double>> currentState;
    std::vector<double> T, t, max_dist, min_dist;

    ros::Publisher initTraj_pub;


    // ROS messages

    visualization_msgs::MarkerArray msgs_initTraj;


    void update_initTraj(){
        visualization_msgs::MarkerArray mk_array;
        for (int qi = 0; qi < qn; qi++) {
            for (int m = 0; m < M+1; m++) {
                visualization_msgs::Marker mk;
                mk.header.frame_id = "world";
                mk.header.stamp = ros::Time::now();
                mk.ns = "mav" + std::to_string(qi);
                mk.type = visualization_msgs::Marker::CUBE;
                mk.action = visualization_msgs::Marker::ADD;

                mk.pose.orientation.x = 0.0;
                mk.pose.orientation.y = 0.0;
                mk.pose.orientation.z = 0.0;
                mk.pose.orientation.w = 1.0;

                mk.color.a = 1.0;
                mk.color.r = param.color[qi][0];
                mk.color.g = param.color[qi][1];
                mk.color.b = param.color[qi][2];

                mk.id = m;
                octomap::point3d p_init = initTrajPlanner_obj->initTraj[qi][m];
                mk.pose.position.x = p_init.x();
                mk.pose.position.y = p_init.y();
                mk.pose.position.z = p_init.z();

                mk.scale.x = 0.1;
                mk.scale.y = 0.1;
                mk.scale.z = 0.1;

                mk_array.markers.emplace_back(mk);
                ROS_INFO(" [my] (%d) (%1.2f, %1.2f, %1.2f), ... ", qi, p_init.x(), p_init.y(), p_init.z());
            }
        }
        msgs_initTraj = mk_array;
    }

};