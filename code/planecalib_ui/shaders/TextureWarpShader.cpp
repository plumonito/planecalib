#include "TextureWarpShader.h"
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

    return res;
}

void TextureWarpShader::setMVPMatrix(const Eigen::Matrix4f &mvp)
{
	//Transpose to opengl column-major format
    glUseProgram(mProgram.getId());
    glUniformMatrix4fv(mUniformMVPMatrix, 1, false, mvp.data());

    glUseProgram(mProgram_Alpha.getId());
    glUniformMatrix4fv(mUniformMVPMatrix_Alpha, 1, false, mvp.data());
}

void TextureWarpShader::renderTexture(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector2i &imageSize,
	const Eigen::Vector2i &screenOrigin)
{
    const float kDepth = 1.0f;
	std::vector<Eigen::Vector4f> vertices;
	vertices.push_back(Eigen::Vector4f(screenOrigin.x() + (float)imageSize.x() - 1, screenOrigin.y(), kDepth, 1.0f));
	vertices.push_back(Eigen::Vector4f(screenOrigin.x(), screenOrigin.y(), kDepth, 1.0f));
	vertices.push_back(Eigen::Vector4f(screenOrigin.x() + (float)imageSize.x() - 1, screenOrigin.y() + (float)imageSize.y() - 1, kDepth, 1.0f));
	vertices.push_back(Eigen::Vector4f(screenOrigin.x(), screenOrigin.y() + (float)imageSize.y() - 1, kDepth, 1.0f));
    
	std::vector<Eigen::Vector2f> textureCoords;
	textureCoords.emplace_back(1, 0);
	textureCoords.emplace_back(0, 0);
	textureCoords.emplace_back(1, 1);
	textureCoords.emplace_back(0, 1);
    renderTexture(GL_TRIANGLE_STRIP, target, id, homography, imageSize, vertices.data(), textureCoords.data(), 4);
}

void TextureWarpShader::renderTexture(GLenum mode, GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector2i &imageSize, const Eigen::Vector4f *vertices,
	const Eigen::Vector2f *textureCoords, int count)
{
    assert(target == GL_TEXTURE_2D);

    glUseProgram(mProgram.getId());

	Eigen::Matrix3fr finalH;
	Eigen::Matrix3fr normHR = Eigen::Matrix3fr::Identity();
	normHR(0, 0) = (float)(imageSize.x() - 1);
	normHR(1, 1) = (float)(imageSize.y() - 1);
	Eigen::Matrix3fr normHL = Eigen::Matrix3fr::Identity();
    normHL(0, 0) = 1.0f / (imageSize.x() - 1);
    normHL(1, 1) = 1.0f / (imageSize.y() - 1);

	Eigen::Matrix3fr finalH_t = (normHL * homography * normHR).transpose();

    glUniformMatrix3fv(mUniformHomographyMatrix, 1, false, finalH_t.data());

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


void TextureWarpShader::renderTexture(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, float alpha, const Eigen::Vector2i &imageSize,
	const Eigen::Vector2i &screenOrigin)
{
    const float kDepth = 1.0f;
	std::vector<Eigen::Vector4f> vertices;
	vertices.push_back(Eigen::Vector4f(screenOrigin.x() + (float)imageSize.x() - 1, screenOrigin.y(), kDepth, 1.0f));
	vertices.push_back(Eigen::Vector4f(screenOrigin.x(), screenOrigin.y(), kDepth, 1.0f));
	vertices.push_back(Eigen::Vector4f(screenOrigin.x() + (float)imageSize.x() - 1, screenOrigin.y() + (float)imageSize.y() - 1, kDepth, 1.0f));
	vertices.push_back(Eigen::Vector4f(screenOrigin.x(), screenOrigin.y() + (float)imageSize.y() - 1, kDepth, 1.0f));

	std::vector<Eigen::Vector2f> textureCoords;
	textureCoords.emplace_back(1, 0);
	textureCoords.emplace_back(0, 0);
	textureCoords.emplace_back(1, 1);
	textureCoords.emplace_back(0, 1);

	renderTexture(GL_TRIANGLE_STRIP, target, id, homography, alpha, imageSize, vertices.data(), textureCoords.data(), 4);
}

void TextureWarpShader::renderTexture(GLenum mode, GLuint target, GLuint id, const Eigen::Matrix3fr &homography, float alpha, const Eigen::Vector2i &imageSize, const Eigen::Vector4f *vertices,
	const Eigen::Vector2f *textureCoords, int count)
{
    assert(target == GL_TEXTURE_2D);

    glUseProgram(mProgram_Alpha.getId());

	Eigen::Matrix3fr finalH;
	Eigen::Matrix3fr normHR = Eigen::Matrix3fr::Identity();
	normHR(0, 0) = (float)(imageSize.x() - 1);
	normHR(1, 1) = (float)(imageSize.y() - 1);
	Eigen::Matrix3fr normHL = Eigen::Matrix3fr::Identity();
    normHL(0, 0) = 1.0f / (imageSize.x() - 1);
    normHL(1, 1) = 1.0f / (imageSize.y() - 1);

    Eigen::Matrix3fr finalH_t = (normHL * homography * normHR).transpose();

    glUniformMatrix3fv(mUniformHomographyMatrix_Alpha, 1, false, finalH_t.data());

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

}
