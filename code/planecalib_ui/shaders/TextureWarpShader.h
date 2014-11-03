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

	void renderTexture(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector2i &imageSize) { renderTexture(target, id, homography, imageSize, Eigen::Vector2i(0, 0)); }
	void renderTexture(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector2i &imageSize,
										const Eigen::Vector2i &screenOrigin);
	void renderTexture(GLenum mode, GLuint target, GLuint id, const Eigen::Matrix3fr &homography, const Eigen::Vector2i &imageSize, const Eigen::Vector4f *vertices,
		const Eigen::Vector2f *textureCoords, int count);

	void renderTexture(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, float alpha, const Eigen::Vector2i &imageSize) { renderTexture(target, id, homography, alpha, imageSize, Eigen::Vector2i(0, 0)); }
	void renderTexture(GLuint target, GLuint id, const Eigen::Matrix3fr &homography, float alpha, const Eigen::Vector2i &imageSize,
										const Eigen::Vector2i &screenOrigin);
	void renderTexture(GLenum mode, GLuint target, GLuint id, const Eigen::Matrix3fr &homography, float alpha, const Eigen::Vector2i &imageSize, const Eigen::Vector4f *vertices,
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
};

}

#endif
