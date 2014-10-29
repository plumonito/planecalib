/*
 * UserInterfaceInfo.cpp
 *
 *  Created on: 3.2.2014
 *      Author: dan
 */

#include "UserInterfaceInfo.h"

namespace planecalib {

std::unique_ptr<UserInterfaceInfo> UserInterfaceInfo::gInstance;

UserInterfaceInfo &UserInterfaceInfo::Instance()
{
	if(gInstance.get() == NULL)
		gInstance.reset(new UserInterfaceInfo());
	return *gInstance;
}

UserInterfaceInfo::UserInterfaceInfo()
{
	memset(mKeyState, 0, sizeof(mKeyState));
	memset(mSpecialKeyState, 0, sizeof(mSpecialKeyState));
}

} /* namespace planecalib */
