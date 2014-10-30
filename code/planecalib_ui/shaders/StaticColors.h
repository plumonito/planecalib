/*
 * StaticColors.h
 *
 *  Created on: 18.2.2014
 *      Author: dan
 */

#ifndef STATICCOLORS_H_
#define STATICCOLORS_H_

#include <Eigen/Dense>

namespace planecalib {

class StaticColors
{
public:
	static Eigen::Vector4f White(float alpha=1.0f) {return Eigen::Vector4f(1,1,1,alpha);}
	static Eigen::Vector4f Black(float alpha=1.0f) {return Eigen::Vector4f(0,0,0,alpha);}
	static Eigen::Vector4f Gray(float alpha=1.0f,float brightness=0.5f) {return Eigen::Vector4f(brightness,brightness,brightness,alpha);}

	static Eigen::Vector4f Red(float alpha=1.0f, float brightness=1.0f) {return Eigen::Vector4f(brightness,0,0,alpha);}
	static Eigen::Vector4f Green(float alpha=1.0f, float brightness=1.0f) {return Eigen::Vector4f(0,brightness,0,alpha);}
	static Eigen::Vector4f Blue(float alpha=1.0f, float brightness=1.0f) {return Eigen::Vector4f(0,0,brightness,alpha);}

	static Eigen::Vector4f Yellow(float alpha=1.0f, float brightness=1.0f) {return Eigen::Vector4f(brightness,brightness,0,alpha);}
	static Eigen::Vector4f Purple(float alpha=1.0f, float brightness=1.0f) {return Eigen::Vector4f(brightness,0,brightness,alpha);}
	static Eigen::Vector4f Cyan(float alpha=1.0f, float brightness=1.0f) {return Eigen::Vector4f(0,brightness,brightness,alpha);}

	static Eigen::Vector4f ChangeAlpha(const Eigen::Vector4f &color, float alpha=1.0f) {return Eigen::Vector4f(color[0],color[1],color[2],alpha);}
private:
	StaticColors() {}
};

} /* namespace planecalib */

#endif /* STATICCOLORS_H_ */
