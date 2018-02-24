#include <string>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"

double imuX,imuY,imuZ;
geometry_msgs::Quaternion tmpss_;
std_msgs::Header headerss_;
void chatterCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
	imuX=msg->orientation.x;imuY=msg->orientation.y;imuZ=msg->orientation.z;
 	tmpss_=msg->orientation;
	headerss_=msg->header;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "car_dagu_publisher");
    ros::NodeHandle n;
    ros::NodeHandle n2;

    ros::Publisher joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 100);
    ros::Subscriber sub = n2.subscribe("imu", 100, chatterCallback);
  

    tf::TransformBroadcaster broadcaster;
    ros::Rate loop_rate(50);

    const double degree = M_PI/180;

    // robot state
    double tilt = 0, tinc = degree, swivel=0, angle=0, height=0, hinc=0.005;

    // message declarations
    sensor_msgs::JointState joint_state;

    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        joint_state.name.resize(6);
        joint_state.position.resize(6);
        joint_state.name[0] ="joint_wheel_right_0";
        joint_state.position[0] = swivel;
        joint_state.name[1] ="joint_wheel_right_1";
        joint_state.position[1] = swivel;
	joint_state.name[2] ="joint_wheel_right_2";
        joint_state.position[2] = swivel;
	joint_state.name[3] ="joint_wheel_left_0";
        joint_state.position[3] = swivel;
        joint_state.name[4] ="joint_wheel_left_1";
        joint_state.position[4] = swivel;
	joint_state.name[5] ="joint_wheel_left_2";
        joint_state.position[5] = swivel;

	tf::Quaternion tmp_;
	tf::quaternionMsgToTF(tmpss_, tmp_);
	tfScalar imu_yaw, imu_pitch, imu_roll;
	tf::Matrix3x3(tmp_).getRPY(imu_roll, imu_pitch, imu_yaw);
	tf::Transform transform;
	transform.setIdentity();
	tf::Quaternion quat;
	tf::Vector3 origin;	
	//origin.setX( cos(angle)*2);origin.setY(sin(angle)*2);origin.setZ(0.1);
	origin.setX(0.21);origin.setY(0.21);origin.setZ(0.1);
	transform.setOrigin(origin);
	quat.setRPY(imu_roll, imu_pitch, imu_yaw);
	transform.setRotation(quat);


      

	ROS_INFO("Imu Orientation x: [%f],   y: [%f],   z: [%f],  swivel: [%f]  ",imuX,imuY,imuZ,swivel);

        joint_pub.publish(joint_state);

	broadcaster.sendTransform(tf::StampedTransform(transform, headerss_.stamp, "odom","footprint"));        
	angle += degree/4;
	swivel += degree*2;
	
        // This will adjust as needed per iteration
	ros::spinOnce();
        loop_rate.sleep();

    }


    return 0;
}
