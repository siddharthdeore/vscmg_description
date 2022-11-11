#ifndef TFBCASTER_H
#define TFBCASTER_H

#pragma once

#include <tf/transform_broadcaster.h>

class TFBCaster
{
private:
    tf::TransformBroadcaster br;
    tf::Transform transform;
    std::string m_fixed_frame;
    std::string m_tf_frame;

public:
    TFBCaster(const std::string fixed_frame = "world", const std::string tf_frame = "base_link")
    {
        m_fixed_frame = fixed_frame;
        m_tf_frame = tf_frame;
        transform.setIdentity();
    }
    ~TFBCaster() {}
    void broadcast(const tf::Vector3 &pos, const tf::Quaternion &quat)
    {
        transform.setOrigin(pos);
        transform.setRotation(quat);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), m_fixed_frame, m_tf_frame));
    }
    void broadcast(const double &x, const double &y, const double &z)
    {
        transform.setOrigin(tf::Vector3(x, y, z));
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), m_fixed_frame, m_tf_frame));
    }
};

#endif