#include <ros/ros.h>
#include <vscmg_description/TFBCaster.h>

#include <thread>
#include <chrono>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "vscmg_base_publisher");
    ros::NodeHandle node_handle;

    // robot floating base transform
    std::shared_ptr<TFBCaster> floating_base_tf = std::make_shared<TFBCaster>("vscmg/tripod", "vscmg/floating_base");

    auto next = std::chrono::high_resolution_clock::now();
    const tf::Vector3 position = tf::Vector3(0, 0, 0);
    while (ros::ok())
    {
        const auto now = ros::Time::now();
        const tf::Quaternion orientation = tf::Quaternion(cos(now.toSec() * 0.5) * 0.1, sin(now.toSec() * 0.5) * 0.1, now.toSec() * 0.3);

        // floating base TF
        floating_base_tf->broadcast(position, orientation);

        ros::spinOnce();
        next = next + std::chrono::milliseconds(10);
        std::this_thread::sleep_until(next);
    }
    return 0;
}
