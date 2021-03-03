#include <ros/ros.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

// TODO: include ROS msg type headers and libraries

class Safety {
// The class that handles emergency braking
private:
    ros::NodeHandle n;
    double speed;
    double threshold;
    // TODO: create ROS subscribers and publishers

    ros::Subscriber laser_sub;
    ros::Subscriber odom_sub;

    ros::Publisher brake_bool;
    ros::Publisher brake;

public:
    Safety() {
        n = ros::NodeHandle();
        speed = 0.0;

        laser_sub = n.subscribe("/scan", 10, &Safety::scan_callback, this);
        odom_sub = n.subscribe("/odom", 10, &Safety::odom_callback, this);

        brake_bool = n.advertise<std_msgs::Bool>("/brake_bool", 10);
        brake = n.advertise<ackermann_msgs::AckermannDriveStamped>("/brake",10);

        threshold = 0.3;
        
    }
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
        // TODO: update current speed
        speed = odom_msg->twist.twist.linear.x;
    }

    void scan_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg) {

        float angle_increment = scan_msg->angle_increment;
        float curr_angle = scan_msg->angle_min;
        float TTC{100000.0},m_TTC;


        for(float range : scan_msg->ranges){
            m_TTC = range/(std::abs(std::cos(curr_angle)*speed));
            if(m_TTC < TTC && m_TTC > 0) TTC = m_TTC;
            curr_angle += angle_increment;
        }

        if(TTC < threshold){
            ROS_INFO("About to Collide: [%f]",TTC);
            std_msgs::Bool bool_msg;
            bool_msg.data = true;
            ackermann_msgs::AckermannDriveStamped steering_msg;
            steering_msg.header = scan_msg->header;
            steering_msg.drive.speed = 0;

            brake_bool.publish(bool_msg);
            brake.publish(steering_msg);
        }

        else{
            std_msgs::Bool bool_msg;
            bool_msg.data = false;
            brake_bool.publish(bool_msg);        
        }
    }

};
int main(int argc, char ** argv) {
    ros::init(argc, argv, "safety_node");
    Safety sn;
    ros::spin();
    return 0;
}