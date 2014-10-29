#ifndef COLOR_SHADER_H_
#define COLOR_SHADER_H_

#include <opencv2/core.hpp>
#include "ShaderProgram.h"

namespace planecalib
{

class ColorShader
{
public:
    ColorShader() {}

    bool init();
    void free() {mProgram.free(); mProgram4.free();}

    //mvp is in normal opencv row-major order
    void setMVPMatrix(const cv::Matx44f &mvp);

    void drawVertices(GLenum mode, const cv::Point2f *vertices, int count, const cv::Vec4f &color);
    void drawVertices(GLenum mode, const cv::Point2f *vertices, const cv::Vec4f *color, int count);
    void drawVertices(GLenum mode, const cv::Vec4f *vertices, int count, const cv::Vec4f &color);
    void drawVertices(GLenum mode, const cv::Vec4f *vertices, const cv::Vec4f *color, int count);
    void drawVertices(GLenum mode, const unsigned int *indices, unsigned int indexCount, const cv::Vec4f *vertices, const cv::Vec4f *color);
    void drawRect(const cv::Point2f center[], int count, const cv::Vec4f &color, float size, float aspect);
    void drawRect(const cv::Point2f center[], const cv::Vec4f color[], int count, float size, float aspect);

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
