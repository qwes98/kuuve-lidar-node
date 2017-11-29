#include <iostream>
#include <cmath>
#include <memory>
#include <array>
using namespace std;

class LaserSpec 
{
public:
	LaserSpec() = default;
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

	template <class T1, class T2> void getBreakpointArray(const T1& raw_laser_data, T2& breakpoint);
	template <class T> void getPreprocessedLaserData(const T& raw_laser_data, T& preprocessed_laser_data);
	template <class T1, class T2> void getBpAndPreprocessedDataArray(const T1& raw_laser_data, T2& breakpoint, T1& preprocessed_laser_data);

	//array size must be defined value (not variable)
	template <std::size_t T> bool getBreakpointArray(const array<double, T>& raw_laser_data, array<bool, T>& breakpoint);
	template <std::size_t T> bool getPreprocessedLaserData(const array<double, T>& raw_laser_data, array<double, T>& preprocessed_laser_data);
	template <std::size_t T> bool getBpAndPreprocessedDataArray(const array<double, T>& raw_laser_data, array<bool, T>& breakpoint, array<double, T>& preprocessed_laser_data);

	const int getLaserNumber() const;
	const int getLaserDegOfView() const;
	const double getLaserResolution() const;
	const int getLambda() const;
	const double getSigma() const;
	
	Abd& setLambda(const int lambda);
	Abd& setSigma(const double sigma);

private:
	const double deg2rad(const double deg) const;
	const double getDmax(const double dist) const;
	const double getPointsDistance(const double dist0, const double dist1) const;
	template <class T1, class T2> void getBreakpointArray_(const T1& raw_laser_data, T2& breakpoint);
	template <class T> void getPreprocessedLaserData_(const T& raw_laser_data, T& preprocessed_laser_data);
	template <class T1, class T2> void getBpAndPreprocessedDataArray_(const T1& raw_laser_data, T2& breakpoint, T1& preprocessed_laser_data);

private:
	LaserSpec laser_spec_;
	int lambda_;
	double sigma_;
};

inline const int Abd::getLaserNumber() const 
{ 
	return laser_spec_.getNumber();
}

inline const int Abd::getLaserDegOfView() const 
{
	return laser_spec_.getDegOfView(); 
}

inline const double Abd::getLaserResolution() const 
{ 
	return laser_spec_.getResolution();
}

inline const int Abd::getLambda() const 
{
	return lambda_; 
}

inline const double Abd::getSigma() const
{
	return sigma_; 
}

inline Abd& Abd::setLambda(const int lambda)
{
	lambda_ = lambda;
	return *this;
}

inline Abd& Abd::setSigma(const double sigma)
{
	sigma_ = sigma;
	return *this;
}

inline const double Abd::deg2rad(const double deg) const
{
	return deg * M_PI / 180;
}

inline const double Abd::getDmax(const double dist) const
{
	return dist * (sin(deg2rad(laser_spec_.getResolution())) / sin(deg2rad(lambda_ - laser_spec_.getResolution()))) + 3*sigma_;
}

inline const double Abd::getPointsDistance(const double dist0, const double dist1) const
{
	return sqrt(pow(dist0, 2) + pow(dist1, 2) - 2 * dist0 * dist1 * cos(deg2rad(laser_spec_.getResolution())));
}

// header-----------------------------------

// cpp------------------------------------
Abd::Abd(const int deg_of_view, const double resolution, const int lambda, const double sigma)
	: laser_spec_(deg_of_view, resolution),
	  lambda_(lambda),
	  sigma_(sigma)
{}
//
//double pointer traits
template <typename T> struct double_pointer_traits {
  static bool const value = false;  
};

template <>
struct double_pointer_traits<std::unique_ptr<double[]>> {
  static bool const value = true;
};

/* operator[] problems
template <>
struct double_pointer_traits<std::shared_ptr<double>> {
  static bool const value = true;
};
*/

template <>
struct double_pointer_traits<double*> {
  static bool const value = true;
};

template <typename T>
struct is_double_pointer {
    static constexpr bool const value = double_pointer_traits<T>::value;
};

//bool pointer traits
template <typename T> struct bool_pointer_traits {
  static bool const value = false;  
};

template <>
struct bool_pointer_traits<std::unique_ptr<bool[]>> {
  static bool const value = true;
};

/*
template <>
struct bool_pointer_traits<std::shared_ptr<bool>> {
  static bool const value = true;
};
*/

template <>
struct bool_pointer_traits<bool*> {
  static bool const value = true;
};

template <typename T>
struct is_bool_pointer {
    static constexpr bool const value = bool_pointer_traits<T>::value;
};

template <class T1, class T2> void Abd::getBreakpointArray(const T1& raw_laser_data, T2& breakpoint)
{
	if(!is_double_pointer<T1>::value || !is_bool_pointer<T2>::value) {
		return;
	}
	getBreakpointArray_(raw_laser_data, breakpoint);
}

template <class T> void Abd::getPreprocessedLaserData(const T& raw_laser_data, T& preprocessed_laser_data)
{
	if(!is_double_pointer<T>::value) {
		return;
	}
	getPreprocessedLaserData_(raw_laser_data, preprocessed_laser_data);
}

template <class T1, class T2> void Abd::getBpAndPreprocessedDataArray(const T1& raw_laser_data, T2& breakpoint, T1& preprocessed_laser_data)
{
	if(!is_double_pointer<T1>::value || !is_bool_pointer<T2>::value) {
		return;
	}
	getBpAndPreprocessedDataArray_(raw_laser_data, breakpoint, preprocessed_laser_data);
}


template <std::size_t T> bool Abd::getBreakpointArray(const array<double, T>& raw_laser_data, array<bool, T>& breakpoint)
{
	getBreakpointArray_(raw_laser_data, breakpoint);
}

template <std::size_t T> bool Abd::getPreprocessedLaserData(const array<double, T>& raw_laser_data, array<double, T>& preprocessed_laser_data)
{
	getPreprocessedLaserData_(raw_laser_data, preprocessed_laser_data);
}

template <std::size_t T> bool Abd::getBpAndPreprocessedDataArray(const array<double, T>& raw_laser_data, array<bool, T>& breakpoint, array<double, T>& preprocessed_laser_data)
{
	getBpAndPreprocessedDataArray_(raw_laser_data, breakpoint, preprocessed_laser_data);
}


template <class T1, class T2> 
void Abd::getBreakpointArray_(const T1& raw_laser_data, T2& breakpoint)
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
void Abd::getPreprocessedLaserData_(const T& raw_laser_data, T& preprocessed_laser_data)
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

template <class T1, class T2> 
void Abd::getBpAndPreprocessedDataArray_(const T1& raw_laser_data, T2& breakpoint, T1& preprocessed_laser_data)
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
			preprocessed_laser_data[j] = 0;
			breakpoint[j] = true; 
			breakpoint[j-1] = true; 
		}
		else {
			preprocessed_laser_data[j] = raw_laser_data[j];
		}
	}
}

/*
//main to test traits and template function
int main() {
	Abd abd(270, 0.25);
	double* double_point_ptr1 = new double[3];
	double_point_ptr1[0] = 1;
	bool* bool_point_ptr1 = new bool[3];
	bool_point_ptr1[0] = true;
	double* double_point_ptr2 = new double[3];
	double_point_ptr2[0] = 1;
	abd.getBreakpointArray(double_point_ptr1, bool_point_ptr1);
	abd.getPreprocessedLaserData(double_point_ptr1, double_point_ptr2);
	abd.getBpAndPreprocessedDataArray(double_point_ptr1, bool_point_ptr1, double_point_ptr2);
	cout << "---------------------" << endl;
	unique_ptr<double[]> double_smart_ptr1(new double[3]);
	double_smart_ptr1[0] = 20;
	unique_ptr<bool[]> bool_smart_ptr2(new bool[3]);
	bool_smart_ptr2[0] = false;
	unique_ptr<double[]> double_smart_ptr2(new double[3]);
	double_smart_ptr2[0] = 20;
	abd.getBreakpointArray(double_smart_ptr1, bool_smart_ptr2);
	abd.getPreprocessedLaserData(double_smart_ptr1, double_smart_ptr2);
	abd.getBpAndPreprocessedDataArray(double_smart_ptr1, bool_smart_ptr2, double_smart_ptr2);
	cout << "---------------------" << endl;
	array<double, 3> double_arr1 = {1,2,3};
	array<bool, 3> bool_arr1 = {true, true, true};
	array<double, 3> double_arr2 = {1,2,3};
	abd.getBreakpointArray(double_arr1, bool_arr1);
	abd.getPreprocessedLaserData(double_arr1, double_arr2);
	abd.getBpAndPreprocessedDataArray(double_arr1, bool_arr1, double_arr2);
}
*/
