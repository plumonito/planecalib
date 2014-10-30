#ifndef COLOR_SHADER_H_
#define COLOR_SHADER_H_

#include <Eigen/Dense>
#include "ShaderProgram.h"

namespace planecalib
{

class ColorShader
{
public:
    ColorShader() {}

    bool init();
    void free() {mProgram.free(); mProgram4.free();}

	void setMVPMatrix(const Eigen::Matrix4f &mvp);

    void drawVertices(GLenum mode, const Eigen::Vector2f *vertices, int count, const Eigen::Vector4f &color);
	void drawVertices(GLenum mode, const Eigen::Vector2f *vertices, const Eigen::Vector4f *color, int count);
    void drawVertices(GLenum mode, const Eigen::Vector4f *vertices, int count, const Eigen::Vector4f &color);
    void drawVertices(GLenum mode, const Eigen::Vector4f *vertices, const Eigen::Vector4f *color, int count);
    void drawVertices(GLenum mode, const unsigned int *indices, unsigned int indexCount, const Eigen::Vector4f *vertices, const Eigen::Vector4f *color);
    void drawRect(const Eigen::Vector2f center[], int count, const Eigen::Vector4f &color, float size, float aspect);
    void drawRect(const Eigen::Vector2f center[], const Eigen::Vector4f color[], int count, float size, float aspect);

protected:
    ShaderProgram mProgram;
    int mUniformMVPMatrix;
    int mAttribPosCoord;
    int mAttribColor;

    ShaderProgram mProgram4;
    int mUniformMVPMatrix4;
    int mAttribPosCoord4;
    int mAttribColor4;
};

}

#endif /* SHADERPROGRAM_H_ */
