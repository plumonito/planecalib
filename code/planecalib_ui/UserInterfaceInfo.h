/*
 * UserInterfaceInfo.h
 *
 *  Created on: 3.2.2014
 *      Author: dan
 */

#ifndef USERINTERFACEINFO_H_
#define USERINTERFACEINFO_H_

#include <memory>
#include <cassert>
#include <opencv2/core.hpp>

namespace planecalib {

class UserInterfaceInfo {
public:
	static UserInterfaceInfo &Instance();

	const cv::Size2i &getScreenSize() const {return mScreenSize;}
	void setScreenSize(const cv::Size2i &sz) {mScreenSize=sz;}

	bool getKeyState(uchar key) const {assert(key<kKeyStateSize); return mKeyState[key];}
	void setKeyState(uchar key, bool state) {assert(key<kKeyStateSize); mKeyState[key] = state;}

	bool getSpecialKeyState(uchar key) const {assert(key<kKeyStateSize); return mSpecialKeyState[key];}
	void setSpecialKeyState(uchar key, bool state) {assert(key<kKeyStateSize); mSpecialKeyState[key] = state;}

protected:
	static std::unique_ptr<UserInterfaceInfo> gInstance;

	cv::Size2i mScreenSize;

	static const int kKeyStateSize=256;
	bool mKeyState[kKeyStateSize];
	bool mSpecialKeyState[kKeyStateSize];

	UserInterfaceInfo();
};

} /* namespace planecalib */

#endif /* USERINTERFACEINFO_H_ */
