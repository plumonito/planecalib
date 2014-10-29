#include "log.h"
#include <iostream>
#include <opencv2/core.hpp>

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
}
