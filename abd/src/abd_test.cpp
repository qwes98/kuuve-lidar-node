#include <ros/ros.h>
#include <abd/Data.h>
#include <abd/Breakpoint.h>
#include <sensor_msgs/LaserScan.h>

#include <iostream>
#include <cmath>
#include <memory>
using namespace std;

class AbdNode
{
private:
	//라이다 스펙
	const int LASER_NUMBER = 1080
	const double LASER_THETA = 0.25

	//실험적으로 얻어야 할 인자값들
	const int LAMBDA = 10			//Testing : LAMBDA 90
	const double SIGMA = 0.03		//Testing : SIGMA 0.005

	ros::NodeHandle n;
	ros::Publisher bp_removed_laser_pub_;
	ros::Publisher is_breakpoint_pub_;
	ros::Subscriber scan_sub_;

	abd::Data bp_removed_laser_msg_;
	abd::Breakpoint is_breakpoint_msg_; 
public:
	AbdNode();
	const double deg2rad(const double& deg) const { return deg * M_PI / 180; }
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan);
};

AbdNode::AbdNode(void)
{
	bp_removed_laser_pub_ = n.advertise<abd::Data>("breakpoint_removed", 100);
	is_breakpoint_pub_ = n.advertise<abd::Breakpoint>("breakpoint", 100);
	scan_sub_ = n.subscribe("/scan", 100, &AbdNode::scanCallback, this);
}

void AbdNode::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan)
{
	unique_ptr<bool[]> p_is_breakpoint(new bool[LASER_NUMBER]); 
	unique_ptr<double[]> p_laser_data(new double[LASER_NUMBER]);
	double d_max = 0.0;
	double nearby_point_dist = 0.0;

    for(int i = 0; i < LASER_NUMBER; i++)
	{
		p_laser_data[i] = scan->ranges[i];
		p_is_breakpoint[i] = false;
	}

	for(int n = 2; n < LASER_NUMBER; n++)
	{
		d_max = p_laser_data[n-1] * (sin(deg2rad(LASER_THETA))/sin(deg2rad(LAMBDA-LASER_THETA))) + 3*SIGMA;
		nearby_point_dist = sqrt(pow(p_laser_data[n-1], 2) + pow(p_laser_data[n], 2) - 2*p_laser_data[n-1]*p_laser_data[n]*cos(deg2rad(LASER_THETA)));
		if(nearby_point_dist > d_max)
		{
			p_is_breakpoint[n] = true;
			p_is_breakpoint[n-1] = true;
		}
	}

	for(int k = 0; k < LASER_NUMBER; k++)
	{
		if(p_is_breakpoint[k] == true)
		{
			bp_removed_laser_msg_.data[k] = 0;
			is_breakpoint_msg_.data[k] = true; 
		}
		else
		{
			bp_removed_laser_msg_.data[k] = p_laser_data[k];
			is_breakpoint_msg_.data[k] = false;
		}
	}

    bp_removed_laser_pub_.publish(bp_removed_laser_msg_);
	is_breakpoint_pub_.publish(is_breakpoint_msg_); 
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "abd_node");
  AbdNode node;

  ros::spin();
}

