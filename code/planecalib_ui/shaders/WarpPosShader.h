#ifndef WARP_POS_SHADER_H_
#define WARP_POS_SHADER_H_

#include <Eigen/Dense>
#include "ShaderProgram.h"

namespace planecalib
{

class WarpPosShader
{
public:
	WarpPosShader() {}

    bool init();
    void free() {mProgram.free();}

	void setMVPMatrix(const Eigen::Matrix4f &mvp);
	void setHomography(const Eigen::Matrix3f &h);

    void drawVertices(GLenum mode, const Eigen::Vector2f *vertices, int count, const Eigen::Vector4f &color);
	void drawVertices(GLenum mode, const Eigen::Vector2f *vertices, const Eigen::Vector4f *color, int count);

protected:
    ShaderProgram mProgram;
    int mUniformMVPMatrix;
	int mUniformHomography;
	int mAttribPosCoord;
    int mAttribColor;
};

}

#endif /* SHADERPROGRAM_H_ */
