
#include "ColorCameraDivisionShader.h"

namespace planecalib
{

bool ColorCameraDivisionShader::init()
{
    bool res = true;

	const char *uniforms[] = { "uPP","uLambda","uFocal","uRt","uMVPMatrix" };
	int *uniformIds[] = {&mUniformPP, &mUniformLambda, &mUniformFocal, &mUniformRt, &mUniformMVPMatrix};
	const int uniformsCount = sizeof(uniforms) / sizeof(uniforms[0]);

	const char *attribs[] =	{ "aPosCoord", "aColor" };
	int *attribIds[] = {&mAttribPosCoord, &mAttribColor };
	const int attribsCount = sizeof(attribs) / sizeof(attribs[0]);
	
	res &= mProgram.load("assets/colorCameraDivision_render.vert", "assets/colorCameraDivision_render.frag", uniforms, uniformIds, uniformsCount,
							 attribs, attribIds, attribsCount);
    return res;
}

void ColorCameraDivisionShader::setMVPMatrix(const Eigen::Matrix4f &mvp)
{
	//Transpose to opengl column-major format
    glUseProgram(mProgram.getId());
    glUniformMatrix4fv(mUniformMVPMatrix, 1, false, mvp.data());
}
void ColorCameraDivisionShader::setCamera(const CameraModel_<DivisionDistortionModel> &camera)
{
	glUseProgram(mProgram.getId());
	glUniform2f(mUniformPP, camera.getPrincipalPoint()[0], camera.getPrincipalPoint()[1]);
	glUniform1f(mUniformLambda, camera.getDistortionModel().getLambda());
	glUniform2f(mUniformFocal, camera.getFocalLength()[0], camera.getFocalLength()[1]);
}

void ColorCameraDivisionShader::setPose(const Eigen::Matrix3fr &poseR, const Eigen::Vector3f &poseT)
{
	Eigen::Matrix4f Rt;
	Rt.block<3, 3>(0, 0) << poseR;
	Rt.block<3, 1>(0, 3) << poseT;
	Rt.row(3) << 0, 0, 0, 1;

	glUseProgram(mProgram.getId());
    glUniformMatrix4fv(mUniformRt, 1, false, Rt.data());
}

void ColorCameraDivisionShader::drawVertices(GLenum mode, const Eigen::Vector4f *vertices, int count, const Eigen::Vector4f &color)
{
	std::vector<Eigen::Vector4f> colors;
    colors.resize(count, color);

    drawVertices(mode, vertices, colors.data(), count);
}

void ColorCameraDivisionShader::drawVertices(GLenum mode, const Eigen::Vector4f *vertices, const Eigen::Vector4f *color, int count)
{
    glUseProgram(mProgram.getId());

    glVertexAttribPointer(mAttribPosCoord, 4, GL_FLOAT, GL_FALSE, 0, vertices);
    glEnableVertexAttribArray(mAttribPosCoord);

    glVertexAttribPointer(mAttribColor, 4, GL_FLOAT, GL_FALSE, 0, color);
    glEnableVertexAttribArray(mAttribColor);

    glDrawArrays(mode, 0, count);

    glDisableVertexAttribArray(mAttribPosCoord);
    glDisableVertexAttribArray(mAttribColor);
}

void ColorCameraDivisionShader::drawVertices(GLenum mode, const unsigned int *indices, unsigned int indexCount, const Eigen::Vector4f *vertices, const Eigen::Vector4f *color)
{
    glUseProgram(mProgram.getId());

    glVertexAttribPointer(mAttribPosCoord, 4, GL_FLOAT, GL_FALSE, 0, vertices);
    glEnableVertexAttribArray(mAttribPosCoord);

    glVertexAttribPointer(mAttribColor, 4, GL_FLOAT, GL_FALSE, 0, color);
    glEnableVertexAttribArray(mAttribColor);

    glDrawElements(mode, indexCount, GL_UNSIGNED_INT, indices);

    glDisableVertexAttribArray(mAttribPosCoord);
    glDisableVertexAttribArray(mAttribColor);
}

}
