#include <ros/ros.h>
#include <min_snap.h>
#include <nav_msgs/Path.h>
#include <chrono>
#include <visualization_msgs/Marker.h>
using namespace std::chrono;

ros::Subscriber waypt_sub;
ros::Publisher trajectory_pub;

int waypoint_num_;
std::vector<Eigen::Vector3d> waypt_list;

bool trajectory_generation_trigger = false;

Eigen::VectorXd evaluatePoly(double time);

void wayptCallback(const nav_msgs::PathConstPtr &msg)
{
    waypoint_num_ = msg->poses.size();
    if (waypoint_num_ < 1)
        return;
    std::cout << "Received " << waypoint_num_ << " waypoints\n";
    waypt_list.clear();
    waypt_list.reserve(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
        waypt_list.emplace_back(msg->poses[i].pose.position.x, msg->poses[i].pose.position.y, 1.0);
    }

    for (auto waypt : waypt_list)
    {
        std::cout << waypt.transpose() << std::endl;
    }

    trajectory_generation_trigger = true;
}

void visualizeTrajectory(ros::Publisher &publisher, const std::vector<double> time_allocation, const std::vector<Eigen::MatrixXd> traj_coefficient)
{
    std::vector<Eigen::Vector3d> traj_pos;

    int segment = time_allocation.size();

    for (int i = 0; i < segment; i++) // for each segment
    {
        Eigen::Vector3d pt;

        for (double t = 0; t < time_allocation[i]; t += 0.01) // for each timestamp in the segment
        {

            Eigen::VectorXd time = evaluatePoly(t);
            for (int j = 0; j < 3; j++) // x,y,z dimension
            {
                pt(j) = time.transpose() * traj_coefficient[j].col(i);
            }
        }
    }
}

// return the 5^th order monomial basis or its derivatives evaluated at given time
Eigen::VectorXd evaluatePoly(double time)
{
    Eigen::VectorXd time_basis; // monomial basis [1, t, t^2, ..., t^5]
    for (int i = 0; i < 6; i++)
    {
        time_basis(i) = pow(time, i);
    }

    return time_basis;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_visualization");
    ros::NodeHandle nh("~");

    waypt_sub =
        nh.subscribe("/waypoints", 1, wayptCallback);

    trajectory_pub = nh.advertise<visualization_msgs::Marker>("/trajectory_vis", 1);

    ros::Rate rate(10.0);

    while (ros::ok())
    {
        if (trajectory_generation_trigger)
        {
            std::cout << "triggered trajectory generation" << std::endl;
            std::cout << "Generating trajectory that goes through " << waypt_list.size() << " waypoints\n";
            const auto close_form_tic = high_resolution_clock::now();
            auto min_snap_close_form = std::make_shared<MIN_SNAP::min_snap_traj>(waypt_list, 3, true);
            // std::shared_ptr<MIN_SNAP::min_snap_traj> min_snap_close_form = std::make_shared<MIN_SNAP::min_snap_traj>(waypt_list, 3, true);
            MIN_SNAP::poly_traj trajectory;
            trajectory = min_snap_close_form->computeTrajectory(waypt_list);

            std::vector<double> time_allocation = trajectory.time_duration;
            std::vector<Eigen::MatrixXd> min_snap_coeff = trajectory.coeff;

            std::vector<Eigen::MatrixXd> min_snap_coeff_reshaped;
            // int reshape_vector_size = min_snap_coeff[0].cols();
            // int segment_number = reshape_vector_size;
            // min_snap_coeff_reshaped.reserve(reshape_vector_size);
            // for (int i = 0; i < reshape_vector_size; i++)
            // {
            //     Eigen::MatrixXd segment_coefficient;

            //     for (int dimension = 0; dimension < min_snap_coeff.size(); dimension++)
            //     {
            //         segment_coefficient.col(dimension) = min_snap_coeff[i].col(dimension);
            //     }

            //     min_snap_coeff_reshaped.emplace_back(segment_coefficient);
            // }

            // visualizeTrajectory(trajectory_pub, time_allocation, min_snap_coeff);

            const auto close_form_toc = high_resolution_clock::now();
            auto close_form_time_lapsed = duration<double>(close_form_toc - close_form_tic).count();
            std::cout << "close form computation time in (ms):\n";
            std::cout << close_form_time_lapsed * 1000 << std::endl;
            for (auto &entry : trajectory.coeff)
            {
                std::cout << entry << std::endl;
            }

            std::cout << "After reshape\n";

            for (auto &entry : min_snap_coeff_reshaped)
            {
                std::cout << entry << std::endl;
            }

            const auto qp_tic = high_resolution_clock::now();
            auto min_snap_qp = std::make_shared<MIN_SNAP::min_snap_traj>(waypt_list, 3, false);
            MIN_SNAP::poly_traj trajectory_qp;
            trajectory_qp = min_snap_qp->computeTrajectory(waypt_list);
            const auto qp_toc = high_resolution_clock::now();
            auto qp_time_lapsed = duration<double>(qp_toc - qp_tic).count();

            std::cout << "QP computation time in (ms):\n";
            std::cout << qp_time_lapsed * 1000 << std::endl;
            for (auto entry : trajectory_qp.coeff)
            {
                std::cout << entry << std::endl;
            }

            trajectory_generation_trigger = false;
        }

        ros::spinOnce();
        rate.sleep();
    }
}