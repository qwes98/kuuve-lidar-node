/**
 *  @file abd.inl
 *
 *  @details inl은 inline의 줄인말로, inline함수와 template함수들의 정의부분을 모아놓은 파일이다
 *  		 이 파일은 abd.h에서 include하여 사용한다
 *
 */

#ifndef ABD_INL
#define ABD_INL

inline const int Abd::getLaserNumber() const 
{ 
	impl_->getLaserNumber();
}

inline const int Abd::getLaserDegOfView() const 
{
	impl_->getLaserDegOfView();
}

inline const double Abd::getLaserResolution() const 
{ 
	impl_->getLaserResolution();
}

inline const int Abd::getLambda() const 
{
	impl_->getLambda();
}

inline const double Abd::getSigma() const
{
	impl_->getSigma();
}

inline Abd& Abd::setLambda(const int lambda)
{
	impl_->setLambda(lambda);
	return *this;
}

inline Abd& Abd::setSigma(const double sigma)
{
	impl_->setSigma(sigma);
	return *this;
}

template <class T1, class T2> void Abd::getBreakpointArray(const T1& raw_laser_data, T2& breakpoint)
{
	impl_->getBreakpointArray(raw_laser_data, breakpoint);
}

template <class T> void Abd::getPreprocessedLaserData(const T& raw_laser_data, T& preprocessed_laser_data)
{
	impl_->getPreprocessedLaserData(raw_laser_data, preprocessed_laser_data);
}

template <class T1, class T2> void Abd::getBpAndPreprocessedDataArray(const T1& raw_laser_data, T2& breakpoint, T1& preprocessed_laser_data)
{
	impl_->getBpAndPreprocessedDataArray(raw_laser_data, breakpoint, preprocessed_laser_data);
}

#endif
