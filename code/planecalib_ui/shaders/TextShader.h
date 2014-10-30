#ifndef TEXT_SHADER_H_
#define TEXT_SHADER_H_

#include <Eigen/Dense>
#include "ShaderProgram.h"

namespace planecalib
{

class TextShader
{
public:
	TextShader() {}

    bool init();
    void free() {mProgram.free();}

	void setMVPMatrix(const Eigen::Matrix4f &mvp);

    void renderText(GLenum mode, GLuint textureId, const Eigen::Vector4f *vertices, const Eigen::Vector2f *textureCoords, int count, const Eigen::Vector4f &color);

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
