/**
 *  @file abd_impl.inl
 *
 *  @details inl은 inline의 줄인말로, inline함수와 template함수들의 정의부분을 모아놓은 파일이다
 *           이 파일은 abd_impl.h에서 include하여 사용한다                 
 *
 */

#ifndef ABD_IMPL_INL
#define ABD_IMPL_INL

// TODO: inl comments - Jiwon Park 2017/12/10
inline const int AbdImpl::getLaserNumber() const 
{ 
	return laser_spec_.getNumber();
}

inline const int AbdImpl::getLaserDegOfView() const 
{
	return laser_spec_.getDegOfView(); 
}

inline const double AbdImpl::getLaserResolution() const 
{ 
	return laser_spec_.getResolution();
}

inline const int AbdImpl::getLambda() const 
{
	return lambda_; 
}

inline const double AbdImpl::getSigma() const
{
	return sigma_; 
}

inline AbdImpl& AbdImpl::setLambda(const int lambda)
{
	lambda_ = lambda;
	return *this;
}

inline AbdImpl& AbdImpl::setSigma(const double sigma)
{
	sigma_ = sigma;
	return *this;
}

inline const double AbdImpl::deg2rad(const double deg) const
{
	return deg * M_PI / 180;
}

/**
 * Abd알고리즘에서 breakpoint를 찾기위해 필요한 임계값(Dmax)를 찾는 함수
 * 자세한것은 https://openi.nlm.nih.gov/detailedresult.php?img=PMC4279490_sensors-14-20400f3&req=4 참조
 *
 */
inline const double AbdImpl::getDmax(const double dist) const
{
	return dist * (sin(deg2rad(laser_spec_.getResolution())) / sin(deg2rad(lambda_ - laser_spec_.getResolution()))) + 3*sigma_;
}

/**
 * laser의 인접한 점들 사이의 거리를 구하는 함수
 * 제2 코사인법칙 사용
 *
 */
inline const double AbdImpl::getPointsDistance(const double dist0, const double dist1) const
{
	return sqrt(pow(dist0, 2) + pow(dist1, 2) - 2 * dist0 * dist1 * cos(deg2rad(laser_spec_.getResolution())));
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

template <class T1, class T2> void AbdImpl::getBreakpointArray(const T1& raw_laser_data, T2& breakpoint)
{
	if(!is_double_pointer<T1>::value)
		throw std::invalid_argument("Invalid argunemt(container) of raw_laser_data in getBreakpointArray");
	if(!is_bool_pointer<T2>::value)
		throw std::invalid_argument("Invalid argument(container) of breakpoint in getBreakpointArray");

	getBreakpointArray_(raw_laser_data, breakpoint);
}

template <class T> void AbdImpl::getPreprocessedLaserData(const T& raw_laser_data, T& preprocessed_laser_data)
{
	if(!is_double_pointer<T>::value)
		throw std::invalid_argument("Invalid argunemt(container) of raw_laser_data and preprocessed_laser_data in getPreprocessedLaserData");
		
	getPreprocessedLaserData_(raw_laser_data, preprocessed_laser_data);
}

template <class T1, class T2> void AbdImpl::getBpAndPreprocessedDataArray(const T1& raw_laser_data, T2& breakpoint, T1& preprocessed_laser_data)
{
	if(!is_double_pointer<T1>::value)
		throw std::invalid_argument("Invalid argunemt(container) of raw_laser_data and preprocessed_laser_data in getBpAndPreprocessedDataArray");
	if(!is_bool_pointer<T2>::value)
		throw std::invalid_argument("Invalid argument(container) of breakpoint in getBpAndPreprocessedDataArray");

	getBpAndPreprocessedDataArray_(raw_laser_data, breakpoint, preprocessed_laser_data);
}

// TODO: fix multiple definition bug
/*
template <> void AbdImpl::getBreakpointArray<std::vector<double>, std::vector<bool>>(const std::vector<double>& raw_laser_data, std::vector<bool>& breakpoint)
{
	if(raw_laser_data.size() != laser_spec_.getNumber()) {
		throw std::invalid_argument("Your first argument(vector) size of getBreakpointArray must be " + std::to_string(laser_spec_.getNumber()));
	}
	if(!breakpoint.empty()) {
		throw std::invalid_argument("Your second argument(vector) of getBreakpointArray must be empty");
	}

	int point_number = laser_spec_.getNumber();
	double d_max = 0.0;
	double nearby_point_dist = 0.0;

    for(int i = 0; i < point_number; i++) {
		breakpoint.push_back(false);
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

template <> void AbdImpl::getPreprocessedLaserData<std::vector<double>>(const std::vector<double>& raw_laser_data, std::vector<double>& preprocessed_laser_data)
{
	if(raw_laser_data.size() != laser_spec_.getNumber()) {
		throw std::invalid_argument("Your first argument(vector) size of getPreprocessedLaserData must be " + std::to_string(laser_spec_.getNumber()));
	}
	if(!preprocessed_laser_data.empty()) {
		throw std::invalid_argument("Your second argument(vector) of getPreprocessedLaserData must be empty");
	}

	int point_number = laser_spec_.getNumber();
	double d_max = 0.0;
	double nearby_point_dist = 0.0;

	for(int j = 0; j < point_number; j++) {
		if(j == 0 || j == 1) {
			preprocessed_laser_data.push_back(raw_laser_data[j]);
			continue;
		}

		d_max = getDmax(raw_laser_data[j-1]);
		nearby_point_dist = getPointsDistance(raw_laser_data[j-1], raw_laser_data[j]);

		if(nearby_point_dist > d_max) {
			preprocessed_laser_data.push_back(0.0);
		}
		else {
			preprocessed_laser_data.push_back(raw_laser_data[j]);
		}
	}
}

template <> void AbdImpl::getBpAndPreprocessedDataArray<std::vector<double>, std::vector<bool>>(const std::vector<double>& raw_laser_data, std::vector<bool>& breakpoint, std::vector<double>& preprocessed_laser_data)
{
	if(raw_laser_data.size() != laser_spec_.getNumber()) {
		throw std::invalid_argument("Your first argument(vector) size of getBpAndPreprocessedDataArray must be " + std::to_string(laser_spec_.getNumber()));
	}
	if(!breakpoint.empty()) {
		throw std::invalid_argument("Your second argument(vector) of getBpAndPreprocessedDataArray must be empty");
	}
	if(!preprocessed_laser_data.empty()) {
		throw std::invalid_argument("Your third argument(vector) of getBpAndPreprocessedDataArray must be empty");
	}

	int point_number = laser_spec_.getNumber();
	double d_max = 0.0;
	double nearby_point_dist = 0.0;

    for(int i = 0; i < point_number; i++) {
		breakpoint.push_back(false);
	}

	for(int j = 2; j < point_number; j++) {
		d_max = getDmax(raw_laser_data[j-1]);
		nearby_point_dist = getPointsDistance(raw_laser_data[j-1], raw_laser_data[j]);

		if(nearby_point_dist > d_max) {
			preprocessed_laser_data.push_back(0.0);
			breakpoint[j] = true; 
			breakpoint[j-1] = true; 
		}
		else {
			preprocessed_laser_data.push_back(raw_laser_data[j]);
		}
	}
}
*/

template <std::size_t T> void AbdImpl::getBreakpointArray(const std::array<double, T>& raw_laser_data, std::array<bool, T>& breakpoint)
{
	if(raw_laser_data.size() != laser_spec_.getNumber()) {
		throw std::invalid_argument("Your first argument(array) size of getBreakpointArray must be " + std::to_string(laser_spec_.getNumber()));
	}
	if(breakpoint.size() != laser_spec_.getNumber()) {
		throw std::invalid_argument("Your second argument(array) size of getBreakpointArray must be " + std::to_string(laser_spec_.getNumber()));
	}

	getBreakpointArray_(raw_laser_data, breakpoint);
}

template <std::size_t T> void AbdImpl::getPreprocessedLaserData(const std::array<double, T>& raw_laser_data, std::array<double, T>& preprocessed_laser_data)
{
	if(raw_laser_data.size() != laser_spec_.getNumber()) {
		throw std::invalid_argument("Your first argument(array) size of getPreprocessedLaserData must be " + std::to_string(laser_spec_.getNumber()));
	}
	if(preprocessed_laser_data.size() != laser_spec_.getNumber()) {
		throw std::invalid_argument("Your second argument(array) size of getPreprocessedLaserData must be " + std::to_string(laser_spec_.getNumber()));
	}

	getPreprocessedLaserData_(raw_laser_data, preprocessed_laser_data);
}

template <std::size_t T> void AbdImpl::getBpAndPreprocessedDataArray(const std::array<double, T>& raw_laser_data, std::array<bool, T>& breakpoint, std::array<double, T>& preprocessed_laser_data)
{
	if(raw_laser_data.size() != laser_spec_.getNumber()) {
		throw std::invalid_argument("Your first argument(array) size of getBpAndPreprocessedDataArray must be " + std::to_string(laser_spec_.getNumber()));
	}
	if(breakpoint.size() != laser_spec_.getNumber()) {
		throw std::invalid_argument("Your second argument(array) size of getBpAndPreprocessedDataArray must be " + std::to_string(laser_spec_.getNumber()));
	}
	if(preprocessed_laser_data.size() != laser_spec_.getNumber()) {
		throw std::invalid_argument("Your third argument(array) size of getBpAndPreprocessedDataArray must be " + std::to_string(laser_spec_.getNumber()));
	}

	getBpAndPreprocessedDataArray_(raw_laser_data, breakpoint, preprocessed_laser_data);
}


template <class T1, class T2> 
void AbdImpl::getBreakpointArray_(const T1& raw_laser_data, T2& breakpoint)
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
void AbdImpl::getPreprocessedLaserData_(const T& raw_laser_data, T& preprocessed_laser_data)
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
			preprocessed_laser_data[j] = 0.0;
		}
		else {
			preprocessed_laser_data[j] = raw_laser_data[j];
		}
	}
}

template <class T1, class T2> 
void AbdImpl::getBpAndPreprocessedDataArray_(const T1& raw_laser_data, T2& breakpoint, T1& preprocessed_laser_data)
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
			preprocessed_laser_data[j] = 0.0;
			breakpoint[j] = true; 
			breakpoint[j-1] = true; 
		}
		else {
			preprocessed_laser_data[j] = raw_laser_data[j];
		}
	}
}

#endif
