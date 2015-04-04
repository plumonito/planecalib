#include "WarpPosShader.h"

namespace planecalib
{

bool WarpPosShader::init()
{
    bool res = true;

	const char *uniforms[] = { "uMVPMatrix", "uHomography" };
	int *uniformIds[] = {&mUniformMVPMatrix, &mUniformHomography};
	const int uniformsCount = sizeof(uniforms) / sizeof(uniforms[0]);

	const char *attribs[] =	{ "aPosCoord", "aColor" };
	int *attribIds[] = {&mAttribPosCoord, &mAttribColor };
	const int attribsCount = sizeof(attribs) / sizeof(attribs[0]);

	res &= mProgram.load("assets/WarpPosColorShader.vert", "assets/WarpPosColorShader.frag", uniforms, uniformIds, uniformsCount,
							 attribs, attribIds, attribsCount);
    return res;
}

void WarpPosShader::setMVPMatrix(const Eigen::Matrix4f &mvp)
{
    glUseProgram(mProgram.getId());
    glUniformMatrix4fv(mUniformMVPMatrix, 1, false, mvp.data());
}

void WarpPosShader::setHomography(const Eigen::Matrix3f &h)
{
	glUseProgram(mProgram.getId());
	glUniformMatrix3fv(mUniformHomography, 1, false, h.data());
}

void WarpPosShader::drawVertices(GLenum mode, const Eigen::Vector2f *vertices, int count, const Eigen::Vector4f &color)
{
    std::vector<Eigen::Vector4f> colors;
    colors.resize(count, color);

    drawVertices(mode, vertices, colors.data(), count);
}

void WarpPosShader::drawVertices(GLenum mode, const Eigen::Vector2f *vertices, const Eigen::Vector4f *color, int count)
{
    glUseProgram(mProgram.getId());

    glVertexAttribPointer(mAttribPosCoord, 2, GL_FLOAT, GL_FALSE, 0, vertices);
    glEnableVertexAttribArray(mAttribPosCoord);

    glVertexAttribPointer(mAttribColor, 4, GL_FLOAT, GL_FALSE, 0, color);
    glEnableVertexAttribArray(mAttribColor);

    glDrawArrays(mode, 0, count);

    glDisableVertexAttribArray(mAttribPosCoord);
    glDisableVertexAttribArray(mAttribColor);
}

}
