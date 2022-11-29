#include <ros/ros.h>
#include <min_snap.h>
#include <nav_msgs/Path.h>
#include <chrono>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
using namespace std::chrono;

ros::Subscriber waypt_sub;
ros::Publisher trajectory_pub, vel_pub, acc_pub;

int waypoint_num_;
std::vector<Eigen::Vector3d> waypt_list;

double interval;
bool verbose;

bool trajectory_generation_trigger = false;

Eigen::VectorXd evaluatePoly(int derivative_order, double time, int polynomial_order);

geometry_msgs::Point vect2Point(const Eigen::Vector3d &vect)
{
    geometry_msgs::Point point;
    point.x = vect.x();
    point.y = vect.y();
    point.z = vect.z();

    return point;
}

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

void visualizeTrajectory(ros::Publisher &publisher_pos, ros::Publisher &publisher_vel, ros::Publisher &publisher_acc, const std::vector<double> time_allocation, const std::vector<Eigen::MatrixXd> traj_coefficient)
{
    std::vector<Eigen::Vector3d> traj_pos, traj_vel, traj_acc;

    int segment = time_allocation.size();

    visualization_msgs::Marker trajectory_pos;

    trajectory_pos.header.frame_id = "/map";
    trajectory_pos.action = visualization_msgs::Marker::ADD;
    trajectory_pos.pose.orientation.w = 1.0;
    trajectory_pos.id = 1;
    trajectory_pos.type = visualization_msgs::Marker::LINE_STRIP;
    trajectory_pos.scale.x = 0.1;
    trajectory_pos.color.b = 1.0;
    trajectory_pos.color.a = 1.0;

    visualization_msgs::MarkerArray trajectory_vel;
    visualization_msgs::MarkerArray trajectory_acc;

    int marker_index = 0;

    for (int i = 0; i < segment; i++) // for each segment
    {
        Eigen::Vector3d pt, vel, acc;

        for (double t = 0; t < time_allocation[i]; t += interval) // for each timestamp in the segment
        {
            Eigen::VectorXd time_pos = evaluatePoly(0, t, 5);
            Eigen::VectorXd time_vel = evaluatePoly(1, t, 5);
            Eigen::VectorXd time_acc = evaluatePoly(2, t, 5);

            pt = (time_pos.transpose() * traj_coefficient[i]).transpose();
            vel = (time_vel.transpose() * traj_coefficient[i]).transpose();
            acc = (time_acc.transpose() * traj_coefficient[i]).transpose();

            traj_pos.emplace_back(pt);
            traj_vel.emplace_back(vel);
            traj_acc.emplace_back(acc);
            trajectory_pos.points.push_back(vect2Point(pt));

            visualization_msgs::Marker node_vel, node_acc;
            node_vel.header.frame_id = node_acc.header.frame_id = "map";
            node_vel.header.stamp = node_acc.header.stamp = ros::Time::now();
            node_vel.id = node_acc.id = marker_index;
            node_vel.type = node_acc.type = visualization_msgs::Marker::ARROW;
            node_vel.action = node_acc.action = visualization_msgs::Marker::ADD;

            geometry_msgs::Point from, to;
            from = vect2Point(pt);
            to = vect2Point(pt + vel);

            node_vel.points.push_back(from);
            node_vel.points.push_back(to);
            // node.scale.x = 0.03;
            // node.scale.y = 0.08;
            // node.scale.z = 0.05;
            // node_vel.scale.x = 0.2;
            // node_vel.scale.y = 0.4;
            // node_vel.scale.z = 0.4;

            node_vel.scale.x = 0.05;
            node_vel.scale.y = 0.1;
            node_vel.scale.z = 0.05;

            node_vel.color.a = node_acc.color.a = 1.0; // Don't forget to set the alpha!

            node_vel.color.r = 1.0;
            node_vel.color.g = 0.0;
            node_vel.color.b = 0.0;

            trajectory_vel.markers.emplace_back(node_vel);

            // acc vectors
            to = vect2Point(pt + acc);
            node_acc.points.push_back(from);
            node_acc.points.push_back(to);
            // node.scale.x = 0.03;
            // node.scale.y = 0.08;
            // node.scale.z = 0.05;
            node_acc.scale.x = 0.05;
            node_acc.scale.y = 0.1;
            node_acc.scale.z = 0.05;
            node_acc.color.r = 0.0;
            node_acc.color.g = 1.0;
            node_acc.color.b = 0.0;

            trajectory_acc.markers.emplace_back(node_acc);

            marker_index++;
        }
    }

    publisher_pos.publish(trajectory_pos);
    publisher_vel.publish(trajectory_vel);
    publisher_acc.publish(trajectory_acc);
    if (verbose)
    {
        std::cout << "position\n";
        for (auto pos : traj_pos)
        {
            std::cout << pos.transpose() << std::endl;
        }

        std::cout << "velocity\n";
        for (auto vel : traj_vel)
        {
            std::cout << vel.transpose() << std::endl;
        }

        std::cout << "acceleration\n";
        for (auto acc : traj_acc)
        {
            std::cout << acc.transpose() << std::endl;
        }
    }
}

// return the 5^th order monomial basis or its derivatives evaluated at given time
Eigen::VectorXd evaluatePoly(int derivative_order, double time, int polynomial_order)
{
    Eigen::VectorXd time_basis = Eigen::MatrixXd::Zero(polynomial_order + 1, 1); // monomial basis [1, t, t^2, ..., t^5]
    switch (derivative_order)
    {
    case 0:
    { /* code */
        for (int i = 0; i < polynomial_order + 1; i++)
        {
            time_basis(i) = pow(time, i);
        }
        break;
    }

    case 1:
    { /* code */
        for (int i = 1; i < polynomial_order + 1; i++)
        {
            time_basis(i) = i * pow(time, i - 1);
        }
        break;
    }

    case 2:
    { /* code */
        for (int i = 2; i < polynomial_order + 1; i++)
        {
            time_basis(i) = i * (i - 1) * pow(time, i - 2);
        }
        break;
    }
    default:
        break;
    }

    return time_basis;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_visualization");
    ros::NodeHandle nh("~");

    nh.param("interval", interval, 0.5);
    nh.param("verbose", verbose, false);

    waypt_sub =
        nh.subscribe("/waypoints", 1, wayptCallback);

    trajectory_pub = nh.advertise<visualization_msgs::Marker>("/trajectory_vis", 1);
    vel_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_vel_vis", 1);
    acc_pub = nh.advertise<visualization_msgs::MarkerArray>("/trajectory_acc_vis", 1);

    ros::Rate rate(10.0);

    while (ros::ok())
    {
        if (trajectory_generation_trigger)
        {
            std::cout << "triggered trajectory generation" << std::endl;
            std::cout << "Generating trajectory that goes through " << waypt_list.size() << " waypoints\n";
            const auto close_form_tic = high_resolution_clock::now();
            auto min_snap_close_form = std::make_shared<MIN_SNAP::min_snap_traj>(waypt_list, 3, true, verbose);
            // std::shared_ptr<MIN_SNAP::min_snap_traj> min_snap_close_form = std::make_shared<MIN_SNAP::min_snap_traj>(waypt_list, 3, true);
            MIN_SNAP::poly_traj trajectory;
            trajectory = min_snap_close_form->computeTrajectory(waypt_list);
            const auto close_form_toc = high_resolution_clock::now();
            auto close_form_time_lapsed = duration<double>(close_form_toc - close_form_tic).count();

            std::vector<double> time_allocation = trajectory.time_duration;

            if (verbose)
            {
                std::cout << "time_allocation\n";
                for (auto time : time_allocation)
                {
                    std::cout << time << std::endl;
                }
            }

            std::vector<Eigen::MatrixXd> min_snap_coeff = trajectory.coeff;

            std::vector<Eigen::MatrixXd> min_snap_coeff_reshaped;
            int reshape_vector_size = min_snap_coeff[0].cols();
            int segment_number = reshape_vector_size;
            min_snap_coeff_reshaped.reserve(reshape_vector_size);

            // each entry of min_snap_coeff_reshaped
            int segment_coefficient_row, segment_coefficient_col;
            segment_coefficient_row = min_snap_coeff[0].rows(); // polynomial_order+1 i.e. number of rows of each entry of min_snap_coeff
            segment_coefficient_col = min_snap_coeff.size();    // dimension

            for (int i = 0; i < reshape_vector_size; i++)
            {
                Eigen::MatrixXd segment_coefficient = Eigen::MatrixXd::Zero(segment_coefficient_row, segment_coefficient_col);

                for (int dimension = 0; dimension < min_snap_coeff.size(); dimension++)
                {
                    segment_coefficient.col(dimension) = min_snap_coeff[dimension].col(i);
                }

                min_snap_coeff_reshaped.emplace_back(segment_coefficient);
            }
            // min_snap_coeff_reshaped is std::vector<Eigen::MatrixXd>
            // each entry of the vector is the polynomial coefficients matrix (3 columns for 3 axis) for one segment
            visualizeTrajectory(trajectory_pub, vel_pub, acc_pub, time_allocation, min_snap_coeff_reshaped);

            std::cout << "close form computation time in (ms):\n";
            std::cout << close_form_time_lapsed * 1000 << std::endl;
            if (verbose)
            {
                for (auto &entry : trajectory.coeff)
                {
                    std::cout << entry << std::endl;
                    std::cout << "-------------\n";
                }

                std::cout << "After reshape\n";

                for (auto &entry : min_snap_coeff_reshaped)
                {
                    std::cout << entry << std::endl;
                    std::cout << "-------------\n";
                }
            }

            const auto qp_tic = high_resolution_clock::now();
            auto min_snap_qp = std::make_shared<MIN_SNAP::min_snap_traj>(waypt_list, 3, false, verbose);
            MIN_SNAP::poly_traj trajectory_qp;
            trajectory_qp = min_snap_qp->computeTrajectory(waypt_list);
            const auto qp_toc = high_resolution_clock::now();
            auto qp_time_lapsed = duration<double>(qp_toc - qp_tic).count();

            std::cout << "QP computation time in (ms):\n";
            std::cout << qp_time_lapsed * 1000 << std::endl;
            if (verbose)
            {
                for (auto entry : trajectory_qp.coeff)
                {
                    std::cout << entry << std::endl;
                }
            }

            trajectory_generation_trigger = false;
        }

        ros::spinOnce();
        rate.sleep();
    }
}