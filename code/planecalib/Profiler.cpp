/*
 * Profiler.cpp
 *
 *  Created on: May 9, 2013
 *      Author: danielh
 */

#include "Profiler.h"

namespace planecalib
{

std::unique_ptr<Profiler> Profiler::gInstance;

ProfilerThreadData::ProfilerThreadData(std::thread::id id):
	mId(id),
	mRootSection(NULL,"root")
{
	mRootSection.setLastTic(Profiler::Instance().now());
	mActiveSections.push(&mRootSection);
}

Profiler::Profiler():
        mShowTotals(false)
{
}

void Profiler::setCurrentThreadName(const std::string &name)
{
	{
		std::lock_guard<std::mutex> lock(mMutex);

		//Search for a thread data with the same name
		for(auto it=mThreads.begin(),end=mThreads.end(); it!=end; ++it)
		{
			ProfilerThreadData &data = *it->second;
			if(data.getName() == name)
			{
				//Match!
				//Update the id
				data.setId(std::this_thread::get_id());

				//Update the thread map
				std::unique_ptr<ProfilerThreadData> dataPtr = std::move(it->second);
				mThreads.erase(it);
				mThreads.emplace(data.getId(), std::move(dataPtr));
				return;
			}
		}
	}

	//Nobody with the same name
	auto &data = *getThreadData();
	data.setName(name);
}

ProfilerThreadData *Profiler::getThreadData()
{
	std::lock_guard<std::mutex> lock(mMutex);

	auto it=mThreads.find(std::this_thread::get_id());
	if(it==mThreads.end())
	{
		ProfilerThreadData *data = new ProfilerThreadData(std::this_thread::get_id());
		mThreads.insert(std::make_pair(data->getId(),std::unique_ptr<ProfilerThreadData>(data)));
		return data;
	}
	else
		return it->second.get();
}

}
