#ifndef TEXTURE_WARP_SHADER_H_
#define TEXTURE_WARP_SHADER_H_

#include <opencv2/core.hpp>
#include "ShaderProgram.h"

namespace planecalib
{

class TextureWarpShader
{
public:
	TextureWarpShader() {}

    bool init();
    void free() {mProgram.free(); mProgram_Alpha.free();}

    void setMVPMatrix(const Eigen::Matrix4f &mvp);

	void renderTexture(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector2i &imageSize) { renderTexture(target, id, homography, imageSize, Eigen::Vector2f(0, 0)); }
	void renderTexture(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector2i &imageSize, const Eigen::Vector2f &screenOrigin);
	void renderTexture(GLenum mode, GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector2i &imageSize, const Eigen::Vector4f *vertices,
		const Eigen::Vector2f *textureCoords, int count);

	//////////////
	//Fixed alpha
	void renderTextureFixedAlpha(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, float alpha, const Eigen::Vector2i &imageSize) { renderTextureFixedAlpha(target, id, homography, alpha, imageSize, Eigen::Vector2f(0, 0)); }
	void renderTextureFixedAlpha(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, float alpha, const Eigen::Vector2i &imageSize, const Eigen::Vector2f &screenOrigin);
	void renderTextureFixedAlpha(GLenum mode, GLuint target, GLuint id, const Eigen::Matrix3fr &homography, float alpha, const Eigen::Vector2i &imageSize, const Eigen::Vector4f *vertices,
										const Eigen::Vector2f *textureCoords, int count);

	//////////////
	//Fixed color
	void renderTextureAsAlpha(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector4f &color, const Eigen::Vector2i &imageSize) { renderTextureAsAlpha(target, id, homography, color, imageSize, Eigen::Vector2f(0, 0)); }
	void renderTextureAsAlpha(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector4f &color, const Eigen::Vector2i &imageSize, const Eigen::Vector2f &screenOrigin);
	void renderTextureAsAlpha(GLenum mode, GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector4f &color, const Eigen::Vector2i &imageSize, const Eigen::Vector4f *vertices,
		const Eigen::Vector2f *textureCoords, int count);

protected:
    ShaderProgram mProgram;
    int mUniformMVPMatrix;
    int mUniformHomographyMatrix;
    int mUniformTexture;
    int mAttribPosCoord;
    int mAttribTexCoord;

    ShaderProgram mProgram_Alpha;
    int mUniformMVPMatrix_Alpha;
    int mUniformHomographyMatrix_Alpha;
    int mUniformTexture_Alpha;
    int mUniformAlpha_Alpha;
    int mAttribPosCoord_Alpha;
    int mAttribTexCoord_Alpha;

	ShaderProgram mProgram_Color;
	int mUniformMVPMatrix_Color;
	int mUniformHomographyMatrix_Color;
	int mUniformTexture_Color;
	int mUniformColor_Color;
	int mAttribPosCoord_Color;
	int mAttribTexCoord_Color;

	Eigen::Matrix3f adjustHomography(const Eigen::Vector2i &imageSize, const Eigen::Matrix3fr &H);
};

}

#endif
