/**
 *  @file laser_spec.h
 *
 */

#ifndef LASER_SPEC_H
#define LASER_SPEC_H

#include <stdexcept>

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
			throw std::invalid_argument("Invalid argument of deg_of_view in LaserSpec class");
		else if(resolution <= 0 || resolution > 360)
			throw std::invalid_argument("Invalid argument of resolution in LaserSpec class");

		point_number_ = deg_of_view / resolution;
		deg_of_view_ = deg_of_view;
		resolution_ = resolution;
	}

	// Getters
	const int getNumber() const { return point_number_; }
	const int getDegOfView() const { return deg_of_view_; }
	const double getResolution() const { return resolution_; }
protected:
	int point_number_; ///< laser point 개수
	int deg_of_view_;
	double resolution_;
};	

#endif
