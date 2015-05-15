#include "TextureWarpShader.h"
#include "TextureShader.h"
#include <cassert>

namespace planecalib
{

bool TextureWarpShader::init()
{
	bool res;

	{
		const char *uniforms[] = { "uMVPMatrix", "uTexture" ,"uHomography"};
		int *uniformIds[] = {&mUniformMVPMatrix, &mUniformTexture, &mUniformHomographyMatrix};
		const int uniformsCount = sizeof(uniforms) / sizeof(uniforms[0]);

		const char *attribs[] = { "aPosCoord", "aTexCoord" };
		int *attribIds[] = {&mAttribPosCoord, &mAttribTexCoord};
		const int attribsCount = sizeof(attribs) / sizeof(attribs[0]);

		res = mProgram.load("assets/texture_warp.vert", "assets/texture_warp.frag", uniforms, uniformIds, uniformsCount, attribs, attribIds,
								attribsCount);
	}
    {
		const char *uniforms[] = { "uMVPMatrix", "uTexture" ,"uHomography","uAlpha"};
		int *uniformIds[] = {&mUniformMVPMatrix_Alpha, &mUniformTexture_Alpha, &mUniformHomographyMatrix_Alpha, &mUniformAlpha_Alpha};
		const int uniformsCount = sizeof(uniforms) / sizeof(uniforms[0]);

		const char *attribs[] = { "aPosCoord", "aTexCoord" };
		int *attribIds[] = {&mAttribPosCoord_Alpha, &mAttribTexCoord_Alpha};
		const int attribsCount = sizeof(attribs) / sizeof(attribs[0]);

		res &= mProgram_Alpha.load("assets/texture_warp.vert", "assets/texture_warp_fixed_alpha.frag", uniforms, uniformIds, uniformsCount, attribs, attribIds,
								attribsCount);
    }
	{
		const char *uniforms[] = { "uMVPMatrix", "uTexture", "uHomography", "uColor" };
		int *uniformIds[] = { &mUniformMVPMatrix_Color, &mUniformTexture_Color, &mUniformHomographyMatrix_Color, &mUniformColor_Color };
		const int uniformsCount = sizeof(uniforms) / sizeof(uniforms[0]);

		const char *attribs[] = { "aPosCoord", "aTexCoord" };
		int *attribIds[] = { &mAttribPosCoord_Color, &mAttribTexCoord_Color };
		const int attribsCount = sizeof(attribs) / sizeof(attribs[0]);

		res &= mProgram_Color.load("assets/texture_warp.vert", "assets/texture_warp_fixed_color.frag", uniforms, uniformIds, uniformsCount, attribs, attribIds,
			attribsCount);
	}

    return res;
}

void TextureWarpShader::setMVPMatrix(const Eigen::Matrix4f &mvp)
{
	//Transpose to opengl column-major format
    glUseProgram(mProgram.getId());
    glUniformMatrix4fv(mUniformMVPMatrix, 1, false, mvp.data());

    glUseProgram(mProgram_Alpha.getId());
    glUniformMatrix4fv(mUniformMVPMatrix_Alpha, 1, false, mvp.data());

	glUseProgram(mProgram_Color.getId());
	glUniformMatrix4fv(mUniformMVPMatrix_Color, 1, false, mvp.data());
}

Eigen::Matrix3f TextureWarpShader::adjustHomography(const Eigen::Vector2i &imageSize, const Eigen::Matrix3fr &H)
{
	Eigen::Matrix3f finalH;
	Eigen::Matrix3fr normHR = Eigen::Matrix3fr::Identity();
	normHR(0, 0) = (float)(imageSize.x() - 1);
	normHR(1, 1) = (float)(imageSize.y() - 1);
	Eigen::Matrix3fr normHL = Eigen::Matrix3fr::Identity();
	normHL(0, 0) = 1.0f / (imageSize.x() - 1);
	normHL(1, 1) = 1.0f / (imageSize.y() - 1);

	return (normHL * H * normHR);
}

void TextureWarpShader::renderTexture(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector2i &imageSize,
	const Eigen::Vector2f &screenOrigin)
{
	std::vector<Eigen::Vector4f> vertices;
	std::vector<Eigen::Vector2f> textureCoords;
	TextureShader::CreateVertices(screenOrigin, imageSize, vertices, textureCoords);
    renderTexture(GL_TRIANGLE_STRIP, target, id, homography, imageSize, vertices.data(), textureCoords.data(), 4);
}

void TextureWarpShader::renderTexture(GLenum mode, GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector2i &imageSize, const Eigen::Vector4f *vertices,
	const Eigen::Vector2f *textureCoords, int count)
{
    assert(target == GL_TEXTURE_2D);

    glUseProgram(mProgram.getId());

	Eigen::Matrix3f finalH;
	finalH = adjustHomography(imageSize, homography);
    glUniformMatrix3fv(mUniformHomographyMatrix, 1, false, finalH.data());

    // setup uniforms
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(target, id);
    glUniform1i(mUniformTexture, 0);

    // drawing quad
    glVertexAttribPointer(mAttribPosCoord, 4, GL_FLOAT, GL_FALSE, 0, vertices);
    glVertexAttribPointer(mAttribTexCoord, 2, GL_FLOAT, GL_FALSE, 0, textureCoords);
    glEnableVertexAttribArray(mAttribPosCoord);
    glEnableVertexAttribArray(mAttribTexCoord);
    glDrawArrays(mode, 0, count);
    glDisableVertexAttribArray(mAttribPosCoord);
    glDisableVertexAttribArray(mAttribTexCoord);
}


void TextureWarpShader::renderTextureFixedAlpha(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, float alpha, const Eigen::Vector2i &imageSize,
	const Eigen::Vector2f &screenOrigin)
{
	std::vector<Eigen::Vector4f> vertices;
	std::vector<Eigen::Vector2f> textureCoords;
	TextureShader::CreateVertices(screenOrigin, imageSize, vertices, textureCoords);
	renderTextureFixedAlpha(GL_TRIANGLE_STRIP, target, id, homography, alpha, imageSize, vertices.data(), textureCoords.data(), 4);
}

void TextureWarpShader::renderTextureFixedAlpha(GLenum mode, GLuint target, GLuint id, const Eigen::Matrix3fr &homography, float alpha, const Eigen::Vector2i &imageSize, const Eigen::Vector4f *vertices,
	const Eigen::Vector2f *textureCoords, int count)
{
    assert(target == GL_TEXTURE_2D);

    glUseProgram(mProgram_Alpha.getId());

	Eigen::Matrix3f finalH;
	finalH = adjustHomography(imageSize, homography);
	glUniformMatrix3fv(mUniformHomographyMatrix_Alpha, 1, false, finalH.data());

    glUniform1f(mUniformAlpha_Alpha, alpha);

    // setup uniforms
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(target, id);
    glUniform1i(mUniformTexture_Alpha, 0);

    // drawing quad
    glVertexAttribPointer(mAttribPosCoord_Alpha, 4, GL_FLOAT, GL_FALSE, 0, vertices);
    glVertexAttribPointer(mAttribTexCoord_Alpha, 2, GL_FLOAT, GL_FALSE, 0, textureCoords);
    glEnableVertexAttribArray(mAttribPosCoord_Alpha);
    glEnableVertexAttribArray(mAttribTexCoord_Alpha);
    glDrawArrays(mode, 0, count);
    glDisableVertexAttribArray(mAttribPosCoord_Alpha);
    glDisableVertexAttribArray(mAttribTexCoord_Alpha);
}


void TextureWarpShader::renderTextureAsAlpha(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector4f &color, const Eigen::Vector2i &imageSize,
	const Eigen::Vector2f &screenOrigin)
{
	std::vector<Eigen::Vector4f> vertices;
	std::vector<Eigen::Vector2f> textureCoords;
	TextureShader::CreateVertices(screenOrigin, imageSize, vertices, textureCoords);
	renderTextureAsAlpha(GL_TRIANGLE_STRIP, target, id, homography, color, imageSize, vertices.data(), textureCoords.data(), 4);
}

void TextureWarpShader::renderTextureAsAlpha(GLenum mode, GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector4f &color, const Eigen::Vector2i &imageSize, const Eigen::Vector4f *vertices,
	const Eigen::Vector2f *textureCoords, int count)
{
	assert(target == GL_TEXTURE_2D);

	glUseProgram(mProgram_Color.getId());

	Eigen::Matrix3f finalH;
	finalH = adjustHomography(imageSize, homography);
	glUniformMatrix3fv(mUniformHomographyMatrix_Color, 1, false, finalH.data());

	glUniform4f(mUniformColor_Color, color[0], color[1], color[2], color[3]);

	// setup uniforms
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(target, id);
	glUniform1i(mUniformTexture_Color, 0);

	// drawing quad
	glVertexAttribPointer(mAttribPosCoord_Color, 4, GL_FLOAT, GL_FALSE, 0, vertices);
	glVertexAttribPointer(mAttribTexCoord_Color, 2, GL_FLOAT, GL_FALSE, 0, textureCoords);
	glEnableVertexAttribArray(mAttribPosCoord_Color);
	glEnableVertexAttribArray(mAttribTexCoord_Color);
	glDrawArrays(mode, 0, count);
	glDisableVertexAttribArray(mAttribPosCoord_Color);
	glDisableVertexAttribArray(mAttribTexCoord_Color);
}

}
