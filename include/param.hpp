#pragma once
#include <ros/ros.h>
#include <ros/package.h>
#include <sp_const.hpp>
#include <string>

namespace MPF{
    class Param {
    public:
        bool log;
        std::string package_path;

        double world_x_min;
        double world_y_min;
        double world_z_min;
        double world_x_max;
        double world_y_max;
        double world_z_max;

        double ecbs_w;
        double grid_xy_res;
        double grid_z_res;
        double grid_margin;

        double box_xy_res;
        double box_z_res;

        double time_step;
        double downwash; // downwash coefficient
        int iteration;
        bool sequential;
        int batch_size; // the number of agents in a batch
        int N_b; // the number of batches
        int n; // degree of polynomial
        int phi; // desired derivatives

        std::vector<std::vector<double>> color;

        bool setROSParam(const ros::NodeHandle &nh);
        void setColor(int qn);
    };

    bool Param::setROSParam(const ros::NodeHandle &nh) {
        nh.param<bool>("log", log, false);

        nh.param<double>("world/x_min", world_x_min, -5);
        nh.param<double>("world/y_min", world_y_min, -5);
        nh.param<double>("world/z_min", world_z_min, 0);
        nh.param<double>("world/x_max", world_x_max, 5);
        nh.param<double>("world/y_max", world_y_max, 5);
        nh.param<double>("world/z_max", world_z_max, 2.5);

        nh.param<double>("grid/xy_res", grid_xy_res, 0.3);
        nh.param<double>("grid/z_res", grid_z_res, 0.6);
        nh.param<double>("grid/margin", grid_margin, 0.2);
        nh.param<double>("ecbs/w", ecbs_w, 1.3);

        /*
        nh.param<double>("box/xy_res", box_xy_res, 0.1);
        nh.param<double>("box/z_res", box_z_res, 0.1);

        nh.param<double>("plan/time_step", time_step, 1);
        nh.param<double>("plan/downwash", downwash, 2.0);
        nh.param<int>("plan/n", n, 5);
        nh.param<int>("plan/phi", phi, 3);
        nh.param<bool>("plan/sequential", sequential, false);
        nh.param<int>("plan/batch_size", batch_size, 4);
        nh.param<int>("plan/N_b", N_b, 0);
        nh.param<int>("plan/iteration", iteration, 1);
*/
        package_path = ros::package::getPath("MPF");

        return true;
    }

    // HSV Colormap
    void Param::setColor(int qn){
        double h,f;
        int i;

        color.resize(qn);
        for(int qi = 0; qi < qn; qi++){
            color[qi].resize(3);
            h = qi * 6 / (double)qn;
            i = (int)h;
            f = h-i;

            switch(i){
                case 0:
                    color[qi][0] = 1;
                    color[qi][1] = f;
                    color[qi][2] = 0;
                    break;
                case 1:
                    color[qi][0] = 1-f;
                    color[qi][1] = 1;
                    color[qi][2] = 0;
                    break;
                case 2:
                    color[qi][0] = 0;
                    color[qi][1] = 1;
                    color[qi][2] = f;
                    break;
                case 3:
                    color[qi][0] = 0;
                    color[qi][1] = 1-f;
                    color[qi][2] = 1;
                    break;
                case 4:
                    color[qi][0]= f;
                    color[qi][1]= 0;
                    color[qi][2]= 1;
                    break;
                case 5:
                    color[qi][0]= 1;
                    color[qi][1]= 0;
                    color[qi][2]= 1-f;
                    break;
                default:
                    break;
            }
        }
    }
}
