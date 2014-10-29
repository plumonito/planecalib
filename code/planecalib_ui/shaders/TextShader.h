#ifndef TEXT_SHADER_H_
#define TEXT_SHADER_H_

#include <opencv2/core.hpp>
#include "ShaderProgram.h"

namespace planecalib
{

class TextShader
{
public:
	TextShader() {}

    bool init();
    void free() {mProgram.free();}

    //mvp is in normal opencv row-major order
    void setMVPMatrix(const cv::Matx44f &mvp);

    void renderText(GLenum mode, GLuint textureId, const cv::Vec4f *vertices, const cv::Vec2f *textureCoords, int count, const cv::Vec4f &color);

protected:
    ShaderProgram mProgram;
    int mUniformMVPMatrix;
    int mUniformTexture;
    int mUniformColor;
    int mAttribPosCoord;
    int mAttribTexCoord;
};

}

#endif /* SHADERPROGRAM_H_ */
