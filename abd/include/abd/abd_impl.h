/**
 *  @file abd_impl.h
 *
 */

#ifndef ABD_IMPL_H
#define ABD_IMPL_H

#include <iostream>
#include <cmath>
#include <memory>
#include <array>
#include <stdexcept>
#include <string>
#include <vector>

#include "laser_spec.h"

/**
 *  @brief   Abd(Adaptive BreakPoint)를 구현한 클래스
 *  @author  Jiwon Park
 *  @details Source of algorithm: https://openi.nlm.nih.gov/detailedresult.php?img=PMC4279490_sensors-14-20400f3&req=4 
 *
 */
class AbdImpl
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
	AbdImpl(const int deg_of_view, const double resolution, const int lambda = 90, const double sigma = 0.005);

	/**
	 *	아래 6개의 public template 함수들은 다양한 컨테이너를 지원하기 위해 만듬 
	 *  현재는 c-array, unique_ptr, std::array 지원
	 *  사용할때에는 한 종류의 컨테이너를 사용해서 원하는 함수를 호출하면 됨
	 *
	 *  기능으로 보면 3개의 종류의 함수가 있음
	 *  1. getBreakpointArray : laser raw데이터가 있는 컨테이너, breakpoint를 저장할 비어있는 컨테이너를 인자로 전달하면 breakpoint를 계산하여 저장
	 *  2. getPreprocessedLaserData : laser raw데이터가 있는 컨테이너, breakpoint를 통해 전처리된 laser 데이터를 저장할 빈 컨테이너를 인자로 전달하면 계산하여 저장
	 *  3. getBpAndPreprocessedDataArray : 1,2번을 한꺼번에 하는 함수
	 *
	 */

	/**
	 *  @brief 	laser raw데이터(거리값)로부터 breakpoint들을 찾아냄
	 *
	 *  @param	raw_laser_data laser raw데이터(거리값) 컨테이너
	 *  @param  breakpoint	   breakpoint 저장할 컨테이너
	 *
     *  @exception 인자로 전달되는 컨테이너가 c-array(pointer) 또는 unique_ptr 또는 std::array가 아닐 시 invalid_argument 객체를 던짐
     *             raw_laser_data와 breakpoint의 컨테이너가 다른 종류일 시 invalid_argument 객체를 던짐
     *
     *  @details Algorithm Complexity: O(N)
	 *
	 */
	template <class T1, class T2> void getBreakpointArray(const T1& raw_laser_data, T2& breakpoint);

	/**
	 *  @brief 	laser raw데이터(거리값)로부터 전처리된 laser 데이터를 만들어냄
	 *
	 *  @param	raw_laser_data          laser raw데이터(거리값) 컨테이너
	 *  @param  preprocessed_laser_data	breakpoint 저장할 컨테이너
	 *
     *  @exception 인자로 전달되는 컨테이너가 c-array(pointer) 또는 unique_ptr 또는 std::array가 아닐 시 invalid_argument 객체를 던짐
     *
     *  @details Algorithm Complexity: O(N)
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
     *  @exception 인자로 전달되는 컨테이너가 c-array(pointer) 또는 unique_ptr 또는 std::array가 아닐 시 invalid_argument 객체를 던짐
     *             raw_laser_data와 breakpoint의 컨테이너가 다른 종류일 시 invalid_argument 객체를 던짐
     *
     *  @details Algorithm Complexity: O(N)
	 *
	 */
	template <class T1, class T2> void getBpAndPreprocessedDataArray(const T1& raw_laser_data, T2& breakpoint, T1& preprocessed_laser_data);

	// specialized(vector argument) template function is in abd_impl.inl

	// template function of array argument
	// array size must be defined value (not variable)
	template <std::size_t T> void getBreakpointArray(const std::array<double, T>& raw_laser_data, std::array<bool, T>& breakpoint);
	template <std::size_t T> void getPreprocessedLaserData(const std::array<double, T>& raw_laser_data, std::array<double, T>& preprocessed_laser_data);
	template <std::size_t T> void getBpAndPreprocessedDataArray(const std::array<double, T>& raw_laser_data, std::array<bool, T>& breakpoint, std::array<double, T>& preprocessed_laser_data);
	
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
	AbdImpl& setLambda(const int lambda);
	AbdImpl& setSigma(const double sigma);

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

#include "./../../src/abd_impl.inl"
#endif
