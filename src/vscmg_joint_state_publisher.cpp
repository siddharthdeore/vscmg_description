#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <urdf/model.h> // urdf parser

#include <vscmg_description/TFBCaster.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <ADCS/Core/Controllers.h>
#include <ADCS/Systems/VSCMG.h>

double drand()
{
    return (2.0 * (double)rand() / RAND_MAX - 1.0);
}
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vscmg_joint_state_publisher");
    ros::NodeHandle node_handle; // two lines to create a publisher object that can talk to ROS

    // robot state publisher
    std::shared_ptr<TFBCaster> floating_base_tf = std::make_shared<TFBCaster>("vscmg/tripod", "vscmg/floating_base");
    ros::Publisher joint_state_publisher = node_handle.advertise<sensor_msgs::JointState>("joint_states", 1000);
    sensor_msgs::JointState joint_state; // create an object of type "joint_state",
    std::string urdf_string = "robot_description";
    urdf::Model model;

    if (!model.initParam(urdf_string))
    {
        ROS_ERROR("Failed to parse urdf file");
        return -1;
    }

    // number of movable joints
    std::size_t N_JOINTS = 0;

    // add all non fixed joints to joint_state
    for (auto joint : model.joints_)
    {
        if (joint.second->type != urdf::Joint::FIXED)
        {
            std::cout << joint.first << std::endl;
            joint_state.name.push_back(joint.first); // Noint Name
            joint_state.position.push_back(0.0);     // Joint Angle
            joint_state.velocity.push_back(0.0);     // Joint Velocity
            joint_state.effort.push_back(0.0);       // Joint Torque
            N_JOINTS++;
        }
    }

    /////

    // satellite object of type VSCMG
    std::shared_ptr<VSCMG> sat = std::make_shared<VSCMG>();

    // initial state
    Eigen::Quaterniond q(drand(), drand(), drand(), drand());
    q.normalize();
    Eigen::Quaterniond qd(1.0, 0, 0, 0);
    Eigen::Matrix<double, 3, 1> w(0.01, 0.01, -0.01);

    // Reaction wheel angular velocities
    Eigen::Matrix<double, 4, 1> Omega(0, 0, 0, 0);

    // gimbal angles
    Eigen::Matrix<double, 4, 1> delta(0, 0, 0, 0);

    VSCMG::state_type X0 = {q.w(), q.x(), q.y(), q.z(), w.x(), w.y(), w.z(), delta[0],
                            delta[1], delta[2], delta[3], Omega[0], Omega[1], Omega[2], Omega[3]};
    sat->set_state(X0);
    sat->set_gimbal_angle(delta);
    sat->set_wheel_velocity(Omega);
    VSCMG::action_type act;
    act.setZero();

    double t = 0;

    /////

    std::mutex qd_mutex; // protects desired quaternion
    auto target_quaternion_callback = [&](const geometry_msgs::Quaternion::ConstPtr &msg)
    {
        // ROS_INFO_STREAM("New Target Quaternion: " << *msg);

        const std::lock_guard<std::mutex> lock(qd_mutex);
        qd.w() = msg->w;
        qd.x() = msg->x;
        qd.y() = msg->y;
        qd.z() = msg->z;
        qd.normalized();
        t = 0.0;
    };

    ros::Subscriber sub = node_handle.subscribe<geometry_msgs::Quaternion>("target_orientation",
                                                         1000,
                                                         target_quaternion_callback);

    //ros::Rate rate(100);
    auto next = std::chrono::high_resolution_clock::now();

    while (ros::ok())
    {
        const auto now = ros::Time::now();
        //////////

        // get current state of Rigid Body (observations)
        VSCMG::state_type X = sat->get_state();

        // attitude quaternion
        q = Eigen::Quaterniond(X[0], X[1], X[2], X[3]);

        // spacecraft angular velocity
        w = Eigen::Vector3d(X[4], X[5], X[6]);

        // calculate error in reference state and current state
        Eigen::Quaterniond qe = get_quaternion_error(q, qd);

        // calcualte control torques which required to bring rigid body in steady state
        Eigen::Vector3d u = -100.0 * qe.vec()*qe.w() - w * 20.0;

        // compute control action from steering law
        act = sat->calc_steering(u, t);

        // integrate a step
        sat->step(act, t, t + 0.01, 0.0001);


        //////////
        // todo update joint states based on dynamics
        joint_state.header.stamp = now;

        // update solution to publish state
        joint_state.position[0] = X[7];
        joint_state.position[1] = X[8];
        joint_state.position[2] = X[9];
        joint_state.position[3] = X[10];

        joint_state.position[4] += X[11] * 0.01;
        joint_state.position[5] += X[12] * 0.01;
        joint_state.position[6] += X[13] * 0.01;
        joint_state.position[7] += X[14] * 0.01;

        // publish the joint states
        joint_state_publisher.publish(joint_state);

        // floating base TF
        const auto position = tf::Vector3(0, 0, 0);
        const auto orientation = tf::Quaternion(q.x(), w.y(), q.z(), q.w());
        floating_base_tf->broadcast(position, orientation);

        ros::spinOnce();
        next = next + std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next);
    }
    return 0;
}
