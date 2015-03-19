#include "log.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <chrono>
#include <ctime>
#include "stdutils.h"

namespace planecalib
{
	std::unique_ptr<MatlabDataLog> MatlabDataLog::gInstance;

	std::mutex Log::gMutex;

	Log::Log(const std::string &file, int line, const std::string &function):
			Log()
	{
	}

	Log::Log(std::ostream &stream):
			mLock(gMutex),
			mStream(stream.rdbuf())
	{
	}

	Log::~Log()
	{
		mStream.flush();
	}

	//Matlab
	MatlabDataLog &MatlabDataLog::Instance()
	{
		if (!gInstance)
		{
			gInstance.reset(new MatlabDataLog());
			gInstance->mStream.open(gInstance->mLogPath + "/" + gInstance->mLogFilename);

			std::string nows = stdutils::TimePointToStr(std::chrono::system_clock::now());
			gInstance->SetValue("varsTime", nows);
		}

		return *gInstance;
	}

}
