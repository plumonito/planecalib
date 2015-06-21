#ifndef COLOR_CAMERA_SHADER_H_
#define COLOR_CAMERA_SHADER_H_

#include <opencv2/core.hpp>
#include "planecalib/CameraModel.h"
#include "ShaderProgram.h"

namespace planecalib
{

class Pose3D;

class ColorCameraShader
{
public:
    ColorCameraShader() {}

    bool init();
    void free() {mProgram.free();}

    void setMVPMatrix(const Eigen::Matrix4f &mvp);
    void setCamera(const CameraModel_<RadialCameraDistortionModel> &camera);
	void setPose(const Eigen::Matrix3fr &poseR, const Eigen::Vector3f &poseT);

	void drawVertices(GLenum mode, const Eigen::Vector4f *vertices, int count, const Eigen::Vector4f &color);
	void drawVertices(GLenum mode, const Eigen::Vector4f *vertices, const Eigen::Vector4f *color, int count);
	void drawVertices(GLenum mode, const unsigned int *indices, unsigned int indexCount, const Eigen::Vector4f *vertices, const Eigen::Vector4f *color);

protected:
    ShaderProgram mProgram;
    int mUniformCameraK;
    int mUniformCameraDist;
    int mUniformCameraMaxRadiusSq;
    int mUniformRt;
    int mUniformMVPMatrix;
    int mAttribPosCoord;
    int mAttribColor;
};

}

#endif /* SHADERPROGRAM_H_ */
