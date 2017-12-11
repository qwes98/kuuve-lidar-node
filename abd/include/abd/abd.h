/**
 *  @file abd.h
 *
 */

#ifndef ABD_H
#define ABD_H

#include <iostream>
#include <cmath>
#include <memory>
#include <array>
#include <stdexcept>

#include "abd_impl.h"

/**
 *  @brief   AbdImpl 클래스의 인터페이스 클래스
 *  @author  Jiwon Park
 *  @details Abd 클래스는 AbdImpl 클래스의 인터페이스 역할을 한다. 사용자는 이 코드만 보고 사용하면 된다. 
 *  		 직접 소스코드를 수정하길 원한다면 AbdImpl 클래스를 수정해야 한다.
 *  		 AbdImpl클래스의 Public 멤버 수정이 있었다면 Abd클래스도 그에따라 수정해주어야한다.
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
	Abd(const Abd& src);
	Abd& operator=(const Abd& rhs);
	~Abd();

	/**
	 *	template 함수들을 통해서 다양한 컨테이너를 지원 
	 *  현재는 c-array, unique_ptr, std::array 사용가능
	 *  사용할때에는 한 종류의 컨테이너를 사용해서 원하는 함수를 호출하면 됨
	 *
	 *  기능으로 보면 3개의 종류의 함수가 있음
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
	AbdImpl* impl_;
};

#include "./../../src/abd.inl"
#endif
