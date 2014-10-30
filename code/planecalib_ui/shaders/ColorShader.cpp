#include "ColorShader.h"

namespace planecalib
{

bool ColorShader::init()
{
    bool res = true;

	const char *uniforms[] = { "uMVPMatrix" };
	int *uniformIds[] = {&mUniformMVPMatrix};
	int *uniformIds4[] = {&mUniformMVPMatrix4};
	const int uniformsCount = sizeof(uniforms) / sizeof(uniforms[0]);

	const char *attribs[] =	{ "aPosCoord", "aColor" };
	int *attribIds[] = {&mAttribPosCoord, &mAttribColor };
	int *attribIds4[] = {&mAttribPosCoord4, &mAttribColor4 };
	const int attribsCount = sizeof(attribs) / sizeof(attribs[0]);

	res &= mProgram.load("assets/color_render.vert", "assets/color_render.frag", uniforms, uniformIds, uniformsCount,
							 attribs, attribIds, attribsCount);
	res &= mProgram4.load("assets/color_render4.vert", "assets/color_render4.frag", uniforms, uniformIds4, uniformsCount,
							 attribs, attribIds4, attribsCount);
    return res;
}

void ColorShader::setMVPMatrix(const Eigen::Matrix4f &mvp)
{
    glUseProgram(mProgram.getId());
    glUniformMatrix4fv(mUniformMVPMatrix, 1, false, mvp.data());

    glUseProgram(mProgram4.getId());
    glUniformMatrix4fv(mUniformMVPMatrix4, 1, false, mvp.data());
}

void ColorShader::drawVertices(GLenum mode, const Eigen::Vector2f *vertices, int count, const Eigen::Vector4f &color)
{
    std::vector<Eigen::Vector4f> colors;
    colors.resize(count, color);

    drawVertices(mode, vertices, colors.data(), count);
}

void ColorShader::drawVertices(GLenum mode, const Eigen::Vector2f *vertices, const Eigen::Vector4f *color, int count)
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

void ColorShader::drawVertices(GLenum mode, const Eigen::Vector4f *vertices, int count, const Eigen::Vector4f &color)
{
    std::vector<Eigen::Vector4f> colors;
    colors.resize(count, color);

    drawVertices(mode, vertices, colors.data(), count);
}

void ColorShader::drawVertices(GLenum mode, const Eigen::Vector4f *vertices, const Eigen::Vector4f *color, int count)
{
    glUseProgram(mProgram4.getId());

    glVertexAttribPointer(mAttribPosCoord4, 4, GL_FLOAT, GL_FALSE, 0, vertices);
    glEnableVertexAttribArray(mAttribPosCoord4);

    glVertexAttribPointer(mAttribColor4, 4, GL_FLOAT, GL_FALSE, 0, color);
    glEnableVertexAttribArray(mAttribColor4);

    glDrawArrays(mode, 0, count);

    glDisableVertexAttribArray(mAttribPosCoord4);
    glDisableVertexAttribArray(mAttribColor4);
}

void ColorShader::drawVertices(GLenum mode, const unsigned int *indices, unsigned int indexCount, const Eigen::Vector4f *vertices, const Eigen::Vector4f *color)
{
    glUseProgram(mProgram4.getId());

    glVertexAttribPointer(mAttribPosCoord4, 4, GL_FLOAT, GL_FALSE, 0, vertices);
    glEnableVertexAttribArray(mAttribPosCoord4);

    glVertexAttribPointer(mAttribColor4, 4, GL_FLOAT, GL_FALSE, 0, color);
    glEnableVertexAttribArray(mAttribColor4);

    glDrawElements(mode, indexCount, GL_UNSIGNED_INT, indices);

    glDisableVertexAttribArray(mAttribPosCoord4);
    glDisableVertexAttribArray(mAttribColor4);
}

void ColorShader::drawRect(const Eigen::Vector2f center[], int count, const Eigen::Vector4f &color, float size, float aspect)
{
    float sizeh = aspect * size / 2;
    float sizev = size / 2;
    float vertex[8];
    Eigen::Vector4f colors[4] = {color,color,color,color};

    glUseProgram(mProgram.getId());
    glVertexAttribPointer(mAttribColor, 4, GL_FLOAT, GL_FALSE, 0, colors);
    glEnableVertexAttribArray(mAttribColor);

    glVertexAttribPointer(mAttribPosCoord, 2, GL_FLOAT, GL_FALSE, 0, vertex);
    glEnableVertexAttribArray(mAttribPosCoord);

    for (int i = 0; i < count; i++)
    {
        const Eigen::Vector2f &c = center[i];
        vertex[0] = c.x() - sizeh;
		vertex[1] = c.y() - sizev;
		vertex[2] = c.x() + sizeh;
		vertex[3] = c.y() - sizev;
		vertex[4] = c.x() + sizeh;
		vertex[5] = c.y() + sizev;
		vertex[6] = c.x() - sizeh;
		vertex[7] = c.y() + sizev;
        glDrawArrays(GL_LINE_LOOP, 0, 4);
    }

    glDisableVertexAttribArray(mAttribPosCoord);
    glDisableVertexAttribArray(mAttribColor);
}

void ColorShader::drawRect(const Eigen::Vector2f center[], const Eigen::Vector4f color[], int count, float size, float aspect)
{
    float sizeh = aspect * size / 2;
    float sizev = size / 2;
    float vertex[8];
    Eigen::Vector4f colors[4];

    glUseProgram(mProgram.getId());
    glVertexAttribPointer(mAttribColor, 4, GL_FLOAT, GL_FALSE, 0, colors);
    glEnableVertexAttribArray(mAttribColor);

    glVertexAttribPointer(mAttribPosCoord, 2, GL_FLOAT, GL_FALSE, 0, vertex);
    glEnableVertexAttribArray(mAttribPosCoord);

    for (int i = 0; i < count; i++)
    {
    	colors[0] = colors[1] = colors[2] = colors[3] = color[i];

        const Eigen::Vector2f &c = center[i];
		vertex[0] = c.x() - sizeh;
		vertex[1] = c.y() - sizev;
		vertex[2] = c.x() + sizeh;
		vertex[3] = c.y() - sizev;
		vertex[4] = c.x() + sizeh;
		vertex[5] = c.y() + sizev;
		vertex[6] = c.x() - sizeh;
		vertex[7] = c.y() + sizev;
        glDrawArrays(GL_LINE_LOOP, 0, 4);
    }

    glDisableVertexAttribArray(mAttribPosCoord);
    glDisableVertexAttribArray(mAttribColor);
}

}
