/**
 *  @file abd_node.cpp
 *
 */
#include <ros/ros.h>
#include <abd/Data.h>
#include <abd/Breakpoint.h>
#include <sensor_msgs/LaserScan.h>

#include "abd/abd.h"
using namespace std;

/**
 *  @brief   Abd알고리즘을 사용하는 Ros Node 클래스
 *  @author  Jiwon Park
 *
 */
class AbdNode
{
public:
	AbdNode();
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan);

private:
	Abd abd_;

	ros::NodeHandle nh_;
	ros::Publisher bp_removed_laser_pub_;
	ros::Publisher is_breakpoint_pub_;
	ros::Subscriber scan_sub_;

	abd::Data bp_removed_laser_msg_;
	abd::Breakpoint is_breakpoint_msg_; 
};

//-------------------------------------------
AbdNode::AbdNode()
	: abd_(270, 0.25)
{
	bp_removed_laser_pub_ = nh_.advertise<abd::Data>("breakpoint_removed", 100);
	is_breakpoint_pub_ = nh_.advertise<abd::Breakpoint>("breakpoint", 100);
	scan_sub_ = nh_.subscribe("/scan", 100, &AbdNode::scanCallback, this);
}

void AbdNode::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan)
{
	const int laser_number = abd_.getLaserNumber();
	try {
		unique_ptr<double[]> raw_laser_ptr(new double[laser_number]);
		unique_ptr<bool[]> breakpoint_ptr(new bool[laser_number]);
		unique_ptr<double[]> preprocessed_laser_ptr(new double[laser_number]);

		for(int i = 0; i < laser_number; i++) {
			raw_laser_ptr[i] = scan->ranges[i];
		}

		//abd_.getBreakpointArray(raw_laser_ptr, breakpoint_ptr);
		//abd_.getPreprocessedLaserData(raw_laser_ptr, preprocessed_laser_ptr);
		abd_.getBpAndPreprocessedDataArray(raw_laser_ptr, breakpoint_ptr, preprocessed_laser_ptr);

		for(int i = 0; i < laser_number; i++) {
			bp_removed_laser_msg_.data[i] = preprocessed_laser_ptr[i];
			is_breakpoint_msg_.data[i] = breakpoint_ptr[i];
		}

		bp_removed_laser_pub_.publish(bp_removed_laser_msg_);
		is_breakpoint_pub_.publish(is_breakpoint_msg_); 
	} catch(const bad_alloc& e) {
		cerr << __FILE__ << "(" << __LINE__
			<< "): Unable to allocate memory!" << endl;
		return;
	}
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "abd_node");
  AbdNode node;

  ros::spin();
}

