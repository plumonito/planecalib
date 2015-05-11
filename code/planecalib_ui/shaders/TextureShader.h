#ifndef TEXTURE_SHADER_H_
#define TEXTURE_SHADER_H_

#include <Eigen/Dense>
#include "ShaderProgram.h"

#include "planecalib/eutils.h"

namespace planecalib
{

class TextureShader
{
public:
	TextureShader() {}

    bool init();
    void free() {mProgram.free(); mProgramFA.free();}

    //mvp is in normal opencv row-major order
	void setMVPMatrix(const Eigen::Matrix4f &mvp);

	static void CreateVertices(const Eigen::Vector2i &imageSize, std::vector<Eigen::Vector4f> &vertices, std::vector<Eigen::Vector2f> &texCoords) { CreateVertices(Eigen::Vector2f(0,0), imageSize, vertices, texCoords); }
	static void CreateVertices(const Eigen::Vector2f &screenOrigin, const Eigen::Vector2i &imageSize, std::vector<Eigen::Vector4f> &vertices, std::vector<Eigen::Vector2f> &texCoords);

	void renderTexture(GLuint target, GLuint id, const Eigen::Vector2i &imageSize) { renderTexture(target, id, imageSize, Eigen::Vector2f(0, 0)); }
	void renderTexture(GLuint target, GLuint id, const Eigen::Vector2i &imageSize,
                                        const Eigen::Vector2f &screenOrigin);
    void renderTexture(GLenum mode, GLuint target, GLuint id, const Eigen::Vector4f *vertices,
                                        const Eigen::Vector2f *textureCoords, int count);
	void drawVertices(GLenum mode, GLuint target, GLuint id, 
									const unsigned int *indices, unsigned int indexCount, 
									const Eigen::Vector4f *vertices, const Eigen::Vector2f *textureCoords);

    void renderTexture(GLuint target, GLuint id, const Eigen::Vector2i &imageSize, float alpha) {renderTexture(target,id,imageSize,Eigen::Vector2f(0,0), alpha);}
	void renderTexture(GLuint target, GLuint id, const Eigen::Vector2i &imageSize,
                                        const Eigen::Vector2f &screenOrigin, float alpha);
    void renderTexture(GLenum mode, GLuint target, GLuint id, const Eigen::Vector4f *vertices,
                                        const Eigen::Vector2f *textureCoords, int count, float alpha);

protected:
    ShaderProgram mProgram;
    int mUniformMVPMatrix;
    int mUniformTexture;
    int mAttribPosCoord;
    int mAttribTexCoord;

    ShaderProgram mProgramFA;
    int mUniformMVPMatrixFA;
    int mUniformTextureFA;
    int mUniformAlphaFA;
    int mAttribPosCoordFA;
    int mAttribTexCoordFA;
};

}

#endif
