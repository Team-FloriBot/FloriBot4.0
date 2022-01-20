#include <switch_nav_base/switch_nav_base.h>

SwitchNavBase::SwitchNavBase()
{
    // constructor
    ptr_tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
}

SwitchNavBase::SwitchNavBase(ros::NodeHandle *nh)
{
    // overloaded constructor
    ptr_tf_listener_ = new tf2_ros::TransformListener(tf_buffer_);
    nh_ = nh;
    seq_ = 0;
    nh->getParam("frame_id_front", frame_id_front_);
    nh->getParam("frame_id_rear", frame_id_rear_);
    nh->getParam("frame_id_output", frame_id_output_);
    nh->getParam("frame_id_map", frame_id_map_);
    frame_id_parent_ = frame_id_front_;
    // default pose of nav_base is identical to frame of front carriage
    tf_nav_base_default_.header.stamp = ros::Time::now();
    tf_nav_base_default_.header.frame_id = frame_id_parent_;
    tf_nav_base_default_.child_frame_id = frame_id_output_;
    tf_nav_base_default_.header.seq = seq_;
    tf_nav_base_default_.transform.translation.x = 0.0;
    tf_nav_base_default_.transform.translation.y = 0.0;
    tf_nav_base_default_.transform.translation.z = 0.0;
    tf_nav_base_default_.transform.rotation.x = 0.0;
    tf_nav_base_default_.transform.rotation.y = 0.0;
    tf_nav_base_default_.transform.rotation.z = 0.0;
    tf_nav_base_default_.transform.rotation.w = 1.0;
    // reset nav_base
    tf_nav_base_ = tf_nav_base_default_;
    create_sub_pub_();
}

SwitchNavBase::~SwitchNavBase()
{
    // destructor
    delete ptr_tf_listener_;
}

void SwitchNavBase::create_sub_pub_()
{   
    // setup /cmd_vel subscriber and timer calling a callback at 20 Hz
    cmd_vel_subscriber_ = nh_->subscribe("/cmd_vel", 1, &SwitchNavBase::cmd_vel_callback_, this);
    body_angle_subscriber_ = nh_->subscribe("/sensors/bodyAngle", 1, &SwitchNavBase::body_angle_callback_, this);
    // tf_timer_ = nh_->createTimer(ros::Duration(0.05), &SwitchNavBase::tf_callback, this);
}

void SwitchNavBase::cmd_vel_callback_(const geometry_msgs::Twist::ConstPtr &msg)
{
    // assign translational speed of robot to variable
    linear_speed_ = msg->linear.x;
}

void SwitchNavBase::body_angle_callback_(const base::Angle::ConstPtr& msg)
{
    bool backward_motion = linear_speed_ < 0.0;
    // reset nav_base from to be identical to frame of front carriage
    tf_nav_base_ = tf_nav_base_default_;
    if (backward_motion)
    {
        ROS_INFO("nav_base switched");
        tf2::Quaternion q_rear_wrt_front;
        float yaw = 0.0;
        float pitch = 0.0;
        float roll = msg->angle;
        q_rear_wrt_front.setEuler(yaw, pitch, roll);
        tf_nav_base_.transform.rotation.x = q_rear_wrt_front[0];
        tf_nav_base_.transform.rotation.y = q_rear_wrt_front[1];
        tf_nav_base_.transform.rotation.z = q_rear_wrt_front[2];
        tf_nav_base_.transform.rotation.w = q_rear_wrt_front[3];
        // tf2::convert(q_rear_wrt_front, tf_nav_base_.transform.rotation);
    }
    // update header information of nav_base frame
    tf_nav_base_.header.stamp = ros::Time::now();
    tf_nav_base_.header.seq = seq_++;
    // publish nav_base frame so it can be used by the local planner
    tf_broadcaster_.sendTransform(tf_nav_base_);
}

// void SwitchNavBase::local_plan_callback_(const nav_msgs::Path::ConstPtr& msg)
// {   
//     // assign local goal (last pose of global path) to pose variable
//     local_goal_pose_in_map_.pose = msg->poses.back().pose;
// }

// void SwitchNavBase::global_goal_status_callback_(const actionlib_msgs::GoalStatusArray::ConstPtr &msg)
// {
//     // assign latest goal status to goal status variable
//     global_goal_status_ = msg->status_list.back().status;
// }

// void SwitchNavBase::tf_callback(const ros::TimerEvent &e)
// {
//     bool exception_caught = true;
//     tryactionlib_msgs
//     {
//         // get the transforms from both rear carriage and map to front carriage frame
//         tf_rear2front_ = tf_buffer_.lookupTransform(frame_id_front_, frame_id_rear_, ros::Time(0));
//         tf_map2front_ = tf_buffer_.lookupTransform(frame_id_front_, frame_id_map_, ros::Time(0));
//         exception_caught = false;
//     }
//     catch (tf2::TransformException &ex)
//     {
//         ROS_WARN("%s", ex.what());
//         ros::Duration(0.1).sleep();
//     }
//     // if transforms received without exceptions
//     if (!exception_caught)
//     {
//         bool backward_motion = linear_speed_ < 0.0;

//         ROS_INFO("switch nav_base: %d", backward_motion);
//         // reset nav_base from to be identical to frame of front carriage
//         tf_nav_base_ = tf_nav_base_default_;
//         if (backward_motion)
//         {
//             // replace the rotation of the nav_base frame 
//             // (which is the same as the front carriage frame by default)
//             // by the rotation of the rear carriage frame w.r.t front carriage frame
//             tf_nav_base_.transform.rotation.x = tf_rear2front_.transform.rotation.x;
//             tf_nav_base_.transform.rotation.y = tf_rear2front_.transform.rotation.y;
//             tf_nav_base_.transform.rotation.z = tf_rear2front_.transform.rotation.z;
//             tf_nav_base_.transform.rotation.w = tf_rear2front_.transform.rotation.w;
//         }
//     }
//     // update header information of nav_base frame
//     tf_nav_base_.header.stamp = ros::Time::now();
//     tf_nav_base_.header.seq = seq_++;
//     // publish nav_base frame so it can be used by the local planner
//     tf_broadcaster_.sendTransform(tf_nav_base_);
// }