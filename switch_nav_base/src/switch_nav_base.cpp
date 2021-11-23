#include <switch_nav_base/switch_nav_base.h>

SwitchNavBase::SwitchNavBase(ros::NodeHandle* nh)
{
    nh_ = nh;
    seq_=0;
    nh->getParam("topic_subscriber", topic_sub_);
    nh->getParam("frame_id_input1", frame_id_input1_);
    nh->getParam("frame_id_input2", frame_id_input2_);
    nh->getParam("frame_id_output", frame_id_output_);
    frame_id_parent_ = frame_id_input1_;
    tf_stamped_.child_frame_id = frame_id_output_;
    tf_stamped_.transform.translation.x = 0.0;
    tf_stamped_.transform.translation.y = 0.0;
    tf_stamped_.transform.translation.z = 0.0;
    tf_stamped_.transform.rotation.x = 0.0;
    tf_stamped_.transform.rotation.y = 0.0;
    tf_stamped_.transform.rotation.z = 0.0;
    tf_stamped_.transform.rotation.w = 1.0;
    create_sub_pub_();
}
SwitchNavBase::~SwitchNavBase(){};

void SwitchNavBase::create_sub_pub_()
{
    cmd_vel_subscriber_=nh_->subscribe(topic_sub_, 1, &SwitchNavBase::cmd_vel_callback_, this);
    tf_timer_=nh_->createTimer(ros::Duration(0.05), &SwitchNavBase::tf_callback, this);
}

void SwitchNavBase::cmd_vel_callback_(const geometry_msgs::Twist::ConstPtr& msg)
{
    linear_speed_ = msg->linear.x;
    frame_id_parent_ = (linear_speed_ < 0.0) ? frame_id_input2_ : frame_id_input1_;
}

void SwitchNavBase::tf_callback(const ros::TimerEvent& e)
{
    tf_stamped_.header.stamp = ros::Time::now();
    tf_stamped_.header.seq = seq_++;
    tf_stamped_.header.frame_id = frame_id_parent_;
    base_link_broadcaster_.sendTransform(tf_stamped_);
}