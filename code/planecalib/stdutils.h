/*
 * stdutils.h
 *
 *  Created on: 2.4.2014
 *      Author: dherrera
 */

#ifndef STDUTILS_H_
#define STDUTILS_H_

#include <future>
#include <ctime>

namespace planecalib
{

class stdutils
{
public:
	template<class T>
	static bool IsTaskRunning(std::future<T> &future);

	template<class TClock>
	static std::string TimePointToStr(const std::chrono::time_point<TClock> &timePoint)
	{
		time_t     now = TClock::to_time_t(timePoint);
		struct tm  tstruct;
		char       buf[80];
		tstruct = *localtime(&now);
		// Visit http://en.cppreference.com/w/cpp/chrono/c/strftime
		// for more information about date/time format
		strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

		return buf;
	}
};

/////////////////////////////////////////////////////////////////////////////////////////
// Implementation
template<class T>
bool stdutils::IsTaskRunning(std::future<T> &future)
{
	if(!future.valid())
		return false;

	auto status = future.wait_for(std::chrono::seconds(0));
	if(status == std::future_status::ready)
		return false;
	else
		return true;
}

}


#endif /* STDUTILS_H_ */
