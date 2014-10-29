/*
 * stdutils.h
 *
 *  Created on: 2.4.2014
 *      Author: dherrera
 */

#ifndef STDUTILS_H_
#define STDUTILS_H_

namespace planecalib
{

class stdutils
{
public:
	template<class T>
	static bool IsTaskRunning(std::future<T> &future);
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
