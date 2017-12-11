/**
 *  @file abd_impl.cpp
 *
 */

#include <iostream>
#include <cmath>
#include <memory>
#include <array>
#include <stdexcept>

#include "abd/abd_impl.h"
using namespace std;

// TODO: cpp comments - Jiwon Park 2017/12/10
AbdImpl::AbdImpl(const int deg_of_view, const double resolution, const int lambda, const double sigma)
	: laser_spec_(deg_of_view, resolution),
	  sigma_(sigma)
{
	// lambda should be bigger than resolution
	if(lambda <= resolution)
		throw invalid_argument("Invalid argument of lambda in Abd class");
	lambda_ = lambda;
}
