/**
 *  @file abd.cpp
 *
 */

#include <iostream>
#include <cmath>
#include <memory>
#include <array>
#include <stdexcept>
using namespace std;

/**
 *  @brief   laser관련 정보를 담고 있는 클래스
 *  @author  Jiwon Park
 *
 */
class LaserSpec 
{
public:
	/** 
	 *  @param     deg_of_view laser 시야각
	 *  @param     resolution  laser 분해능
	 *  @exception deg_of_view 또는 resolution 값이 0(deg) 이하 또는 360(deg) 초과일시 invalid_argument 객체를 던짐
	 *
	 */
	LaserSpec(const int deg_of_view, const double resolution)
	{
		if(deg_of_view <= 0 || deg_of_view > 360)
			throw invalid_argument("Invalid argument of deg_of_view in LaserSpec class");
		else if(resolution <= 0 || resolution > 360)
			throw invalid_argument("Invalid argument of resolution in LaserSpec class");

		point_number_ = deg_of_view / resolution;
		deg_of_view_ = deg_of_view;
		resolution_ = resolution;
	}

	// Getters
	const int getNumber() const { return point_number_; }
	const int getDegOfView() const { return deg_of_view_; }
	const double getResolution() const { return resolution_; }
private:
	int point_number_; ///< laser point 개수
	int deg_of_view_;
	double resolution_;
};	

/**
 *  @brief   ABD(Adaptive BreakPoint)를 구현한 클래스
 *  @author  Jiwon Park
 *  @details Source of algorithm: https://openi.nlm.nih.gov/detailedresult.php?img=PMC4279490_sensors-14-20400f3&req=4 
 *
 */
class Abd
{
public:
	/** 
	 *  @param     deg_of_view laser 시야각
	 *  @param     resolution  laser 분해능
	 *  @param	   lambda	   Dmax를 결정하는 첫번째 인자
	 *  @param	   sigma	   Dmax를 결정하는 두번째 인자
	 *  @exception lambda값이 resolution값보다 작을 시 invalid_argument 객체를 던짐
	 *
	 */
	Abd(const int deg_of_view, const double resolution, const int lambda = 90, const double sigma = 0.005);

	/**
	 *	아래 6개의 public template 함수들은 다양한 컨테이너를 지원하기 위해 만듬 
	 *  현재는 c-array, unique_ptr, std::array 지원
	 *  사용할때에는 한 종류의 컨테이너를 사용해서 원하는 함수를 호출하면 됨
	 *
	 *  기능으로 따지면 3개의 종류의 함수가 있음
	 *  1. getBreakpointArray : laser raw데이터가 있는 컨테이너, breakpoint를 저장할 비어있는 컨테이너를 인자로 전달하면 breakpoint를 계산하여 저장
	 *  2. getPreprocessedLaserData : laser raw데이터가 있는 컨테이너, breakpoint를 통해 전처리된 laser 데이터를 저장할 빈 컨테이너를 인자로 전달하면 계산하여 저장
	 *  3. getBpAndPreprocessedDataArray : 1,2번을 한꺼번에 하는 함수
	 *
	 *
	 *  @exception 인자로 전달되는 컨테이너가 c-array(pointer) 또는 unique_ptr 또는 std::array가 아닐 시 invalid_argument 객체를 던짐
	 *
	 */

	/**
	 *  @brief 	laser raw데이터(거리값)로부터 breakpoint들을 찾아냄
	 *
	 *  @param	raw_laser_data laser raw데이터(거리값) 컨테이너
	 *  @param  breakpoint	   breakpoint 저장할 컨테이너
	 *
	 */
	template <class T1, class T2> void getBreakpointArray(const T1& raw_laser_data, T2& breakpoint);

	/**
	 *  @brief 	laser raw데이터(거리값)로부터 전처리된 laser 데이터를 만들어냄
	 *
	 *  @param	raw_laser_data          laser raw데이터(거리값) 컨테이너
	 *  @param  preprocessed_laser_data	breakpoint 저장할 컨테이너
	 *
	 */
	template <class T> void getPreprocessedLaserData(const T& raw_laser_data, T& preprocessed_laser_data);

	/**
	 *  @brief 	laser raw데이터(거리값)로부터 breakpoint들을 찾아내고 전처리된 laser 데이터를 만들어냄
	 *
	 *  @param	raw_laser_data  	    laser raw데이터(거리값) 컨테이너
	 *  @param  breakpoint	            breakpoint 저장할 컨테이너
	 *  @param  preprocessed_laser_data	breakpoint 저장할 컨테이너
	 *
	 */
	template <class T1, class T2> void getBpAndPreprocessedDataArray(const T1& raw_laser_data, T2& breakpoint, T1& preprocessed_laser_data);

	// array size must be defined value (not variable)
	// 위의 함수들과 동일
	template <std::size_t T> bool getBreakpointArray(const array<double, T>& raw_laser_data, array<bool, T>& breakpoint);
	template <std::size_t T> bool getPreprocessedLaserData(const array<double, T>& raw_laser_data, array<double, T>& preprocessed_laser_data);
	template <std::size_t T> bool getBpAndPreprocessedDataArray(const array<double, T>& raw_laser_data, array<bool, T>& breakpoint, array<double, T>& preprocessed_laser_data);
//TODO: add get function for vector - Jiwon Park 2017/12/10

	// Getters
	const int getLaserNumber() const;
	const int getLaserDegOfView() const;
	const double getLaserResolution() const;
	const int getLambda() const;
	const double getSigma() const;
	
	// Setters
	/**
	 *  @brief setter of lambda
	 *
	 *  @exception lambda값이 resolution값보다 작을 시 invalid_argument 객체를 던짐
	 *
	 */
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
	LaserSpec laser_spec_;	///< Abd를 적용할 라이다의 스펙
	int lambda_;	    	///< Dmax를 결정하는 첫번째 인자
	double sigma_;			///< Dmax를 결정하는 두번째 인자
};


//-------------------------------------------------------------------
//inline method implementations

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

/**
 * Abd알고리즘에서 breakpoint를 찾기위해 필요한 임계값(Dmax)를 찾는 함수
 * 자세한것은 https://openi.nlm.nih.gov/detailedresult.php?img=PMC4279490_sensors-14-20400f3&req=4 참조
 *
 */
inline const double Abd::getDmax(const double dist) const
{
	return dist * (sin(deg2rad(laser_spec_.getResolution())) / sin(deg2rad(lambda_ - laser_spec_.getResolution()))) + 3*sigma_;
}

/**
 * laser의 인접한 점들 사이의 거리를 구하는 함수
 * 제2 코사인법칙 사용
 *
 */
inline const double Abd::getPointsDistance(const double dist0, const double dist1) const
{
	return sqrt(pow(dist0, 2) + pow(dist1, 2) - 2 * dist0 * dist1 * cos(deg2rad(laser_spec_.getResolution())));
}

// header-----------------------------------

// cpp------------------------------------
// TODO: cpp comments - Jiwon Park 2017/12/10
Abd::Abd(const int deg_of_view, const double resolution, const int lambda, const double sigma)
	: laser_spec_(deg_of_view, resolution),
	  sigma_(sigma)
{
	// lambda should be bigger than resolution
	if(lambda <= resolution)
		throw invalid_argument("Invalid argument of lambda in Abd class");
	lambda_ = lambda;
}

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
	if(!is_double_pointer<T1>::value)
		throw invalid_argument("Invalid argunemt(container) of raw_laser_data in Abd::getBreakpointArray");
	if(!is_bool_pointer<T2>::value)
		throw invalid_argument("Invalid argument(container) of breakpoint in Abd::getBreakpointArray");

	getBreakpointArray_(raw_laser_data, breakpoint);
}

template <class T> void Abd::getPreprocessedLaserData(const T& raw_laser_data, T& preprocessed_laser_data)
{
	if(!is_double_pointer<T>::value)
		throw invalid_argument("Invalid argunemt(container) of raw_laser_data and preprocessed_laser_data in Abd::getPreprocessedLaserData");
		
	getPreprocessedLaserData_(raw_laser_data, preprocessed_laser_data);
}

template <class T1, class T2> void Abd::getBpAndPreprocessedDataArray(const T1& raw_laser_data, T2& breakpoint, T1& preprocessed_laser_data)
{
	if(!is_double_pointer<T1>::value)
		throw invalid_argument("Invalid argunemt(container) of raw_laser_data and preprocessed_laser_data in Abd::getBpAndPreprocessedDataArray");
	if(!is_bool_pointer<T2>::value)
		throw invalid_argument("Invalid argument(container) of breakpoint in Abd::getBpAndPreprocessedDataArray");

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
