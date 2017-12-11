/**
 *  @file abd.cpp
 *
 */

#include <iostream>
#include <cmath>
#include <memory>
#include <array>
#include <stdexcept>

#include "abd/abd.h"
using namespace std;

// TODO: cpp comments - Jiwon Park 2017/12/10
Abd::Abd(const int deg_of_view, const double resolution, const int lambda, const double sigma)
{
	impl_ = new AbdImpl(deg_of_view, resolution, lambda, sigma);
}

Abd::Abd(const Abd& src)
{
	impl_ = new AbdImpl(*(src.impl_));
}

Abd& Abd::operator=(const Abd& rhs)
{
	*impl_ = *(rhs.impl_);
	return *this;
}

Abd::~Abd()
{
	delete impl_;
	impl_ = nullptr;
}
