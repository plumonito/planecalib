#include "TextShader.h"

namespace planecalib
{

bool TextShader::init()
{
    const char *uniforms[] = { "uMVPMatrix", "uTexture", "uColor" };
	int *uniformIds[] = {&mUniformMVPMatrix, &mUniformTexture, &mUniformColor};
    const int uniformsCount = sizeof(uniforms) / sizeof(uniforms[0]);

    const char *attribs[] = { "aPosCoord", "aTexCoord" };
	int *attribIds[] = {&mAttribPosCoord, &mAttribTexCoord};
    const int attribsCount = sizeof(attribs) / sizeof(attribs[0]);

    return mProgram.load("assets/text_render.vert", "assets/text_render.frag", uniforms, uniformIds, uniformsCount, attribs, attribIds,
                            attribsCount);
}

void TextShader::setMVPMatrix(const Eigen::Matrix4f &mvp)
{
	//Transpose to opengl column-major format
    glUseProgram(mProgram.getId());
    glUniformMatrix4fv(mUniformMVPMatrix, 1, false, mvp.data());
}

void TextShader::renderText(GLenum mode, GLuint textureId, const Eigen::Vector4f *vertices, const Eigen::Vector2f *textureCoords,
		int count, const Eigen::Vector4f &color)
{
    glUseProgram(mProgram.getId());

    // setup uniforms
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, textureId);
    glUniform1i(mUniformTexture, 0);
    glUniform4f(mUniformColor, color[0], color[1], color[2], color[3]);

    // drawing quad
    glVertexAttribPointer(mAttribPosCoord, 4, GL_FLOAT, GL_FALSE, 0, vertices);
    glVertexAttribPointer(mAttribTexCoord, 2, GL_FLOAT, GL_FALSE, 0, textureCoords);
    glEnableVertexAttribArray(mAttribPosCoord);
    glEnableVertexAttribArray(mAttribTexCoord);
    glDrawArrays(mode, 0, count);
    glDisableVertexAttribArray(mAttribPosCoord);
    glDisableVertexAttribArray(mAttribTexCoord);
}

}
