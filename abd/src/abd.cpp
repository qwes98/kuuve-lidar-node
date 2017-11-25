#include <iostream>
#include <cmath>
#include <memory>
#include <array>
using namespace std;

class LaserSpec 
{
public:
	LaserSpec(const int deg_of_view, const double resolution)
		: POINT_NUMBER(deg_of_view / resolution),
		  DEG_OF_VIEW(deg_of_view),
		  RESOLUTION(resolution)
	{}

	const int getNumber() const { return POINT_NUMBER; }
	const int getDegOfView() const { return DEG_OF_VIEW; }
	const double getResolution() const { return RESOLUTION; }
private:
	const int POINT_NUMBER;
	const int DEG_OF_VIEW;
	const double RESOLUTION;
};	

class Abd
{
public:
	Abd(const int deg_of_view, const double resolution, const int lambda = 90, const double sigma = 0.005);
	void getBreakpointArray(const double* const raw_laser_data, bool* const breakpoint);
	void getPreprocessedLaserData(const double* const raw_laser_data, double* const preprocessed_laser_data);
	void getBpAndPreprocessedDataArray(const double* const raw_laser_data, bool* const breakpoint, double* const preprocessed_laser_data);

	void getBreakpointArray(unique_ptr<double[]>& raw_laser_data, unique_ptr<bool[]>& breakpoint);
	void getPreprocessedLaserData(unique_ptr<double[]>& raw_laser_data, unique_ptr<double[]>& preprocessed_laser_data);
	template <std::size_t T> bool getBreakpointArray(array<double, T>& raw_laser_data, array<bool, T>& breakpoint);
	template <std::size_t T> bool getPreprocessedLaserData(array<double, T>& raw_laser_data, array<double, T>& preprocessed_laser_data);

	bool setLambda(const int lambda);
	bool setSigma(const double sigma);

private:
	const double deg2rad(const double deg) const;
	const double getDmax(const double dist) const;
	const double getPointsDistance(const double dist0, const double dist1) const;
	template <class T> void getBreakpointArray_(T raw_laser_data, T breakpoint);
	template <class T> void getPreprocessedLaserData_(T raw_laser_data, T preprocessed_laser_data);
	template <class T> void getBpAndPreprocessedDataArray_(T raw_laser_data, T breakpoint, T preprocessed_laser_data);

private:
	LaserSpec laser_spec_;
	int lambda_;
	double sigma_;
};

Abd::Abd(const int deg_of_view, const double resolution, const int lambda = 90, const double sigma = 0.005)
	: laser_spec_(deg_of_view, resolution),
	  lambda_(lambda),
	  sigma_(sigma)
{}

void Abd::getBreakpointArray(const double* const raw_laser_data, bool* const breakpoint)
{

}
void Abd::getPreprocessedLaserData(const double* const raw_laser_data, double* const preprocessed_laser_data);
void getBreakpointArray(unique_ptr<double[]>& raw_laser_data, unique_ptr<bool[]>& breakpoint);
void getPreprocessedLaserData(unique_ptr<double[]>& raw_laser_data, unique_ptr<double[]>& preprocessed_laser_data);
template <std::size_t T> bool getBreakpointArray(array<double, T>& raw_laser_data, array<bool, T>& breakpoint);
template <std::size_t T> bool getPreprocessedLaserData(array<double, T>& raw_laser_data, array<double, T>& preprocessed_laser_data);
bool Abd::setLambda(const int lambda);
bool Abd::setSigma(const double sigma);

const double Abd::deg2rad(const double deg) const
{
	return deg * M_PI / 180;
}

const double Abd::getDmax(const double dist) const
{
	return dist * (sin(deg2rad(laser_spec_.getResolution())) / sin(deg2rad(lambda_ - laser_spec_.getResolution()))) + 3*sigma_;
}

const double Abd::getPointsDistance(const double dist0, const double dist1) const
{
	return sqrt(pow(dist0, 2) + pow(dist1, 2) - 2 * dist0 * dist1 * cos(deg2rad(laser_spec_.getResolution())));
}

template <class T> 
void Abd::getBreakpointArray_(T raw_laser_data, T breakpoint)
{
	int point_number = laser_spec_.getNumber();
	double d_max = 0.0;
	double nearby_point_dist = 0.0;

    for(int i = 0; i < point_number; i++) {
		breakpoint[i] = false;
	}

	for(int j = 2; j < point_number; j++) {
		d_max = getDmax(raw_laser_data[j-1]);
		nearby_point_dist = getPointsDistance(raw_laser_data[j-1], raw_laser_data[j]);

		if(nearby_point_dist > d_max) {
			breakpoint[j] = true; 
			breakpoint[j-1] = true; 
		}
	}
}

template <class T>
void Abd::getPreprocessedLaserData_(T raw_laser_data, T preprocessed_laser_data)
{
	int point_number = laser_spec_.getNumber();
	double d_max = 0.0;
	double nearby_point_dist = 0.0;

	for(int j = 0; j < point_number; j++) {
		if(j == 0 || j == 1) {
			preprocessed_laser_data[j] = raw_laser_data[j];
			continue;
		}

		d_max = getDmax(raw_laser_data[j-1]);
		nearby_point_dist = getPointsDistance(raw_laser_data[j-1], raw_laser_data[j]);

		if(nearby_point_dist > d_max) {
			preprocessed_laser_data[j] = 0;
		}
		else {
			preprocessed_laser_data[j] = raw_laser_data[j];
		}
	}
}


//------------------------------------------------------------------------

class AbdNode
{
private:
	//라이다 스펙
	const int LASER_NUMBER = 1080;
	const double LASER_THETA = 0.25;

	//실험적으로 얻어야 할 인자값들
	const int LAMBDA = 10;			//Testing : LAMBDA 90
	const double SIGMA = 0.03;		//Testing : SIGMA 0.005

public:
	AbdNode();
	void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan);
};

void AbdNode::scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan)
{
	double d_max = 0.0;
	double nearby_point_dist = 0.0;

    for(int i = 0; i < LASER_NUMBER; i++)
	{
		is_breakpoint_msg_.data[i] = false;
	}

	for(int n = 2; n < LASER_NUMBER; n++)
	{
		d_max = scan->ranges[n-1] * (sin(deg2rad(LASER_THETA))/sin(deg2rad(LAMBDA-LASER_THETA))) + 3*SIGMA;
		nearby_point_dist = sqrt(pow(scan->ranges[n-1], 2) + pow(scan->ranges[n], 2) - 2*scan->ranges[n-1]*scan->ranges[n]*cos(deg2rad(LASER_THETA)));
		if(nearby_point_dist > d_max)
		{
			bp_removed_laser_msg_.data[n] = 0;
			is_breakpoint_msg_.data[n] = true; 
			is_breakpoint_msg_.data[n-1] = true; 
		}
		else 
		{
			
			bp_removed_laser_msg_.data[n] = scan->ranges[n];
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

