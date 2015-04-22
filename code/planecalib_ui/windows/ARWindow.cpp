
#include "ARWindow.h"
#include "planecalib/PlaneCalibSystem.h"
#include "planecalib/HomographyCalibration.h"
#include "planecalib/Map.h"
#include "planecalib/PoseTracker.h"
#include "../PlaneCalibApp.h"

namespace planecalib
{
void ARWindow::showHelp() const
{
	BaseWindow::showHelp();
	MYAPP_LOG << "Shows an overlay of the triangulated features and the cube."
		"Three display modes: show matches, show reprojected features (only the stable ones with 3 measurements), and show all reprojected features.\n";
}

bool ARWindow::init(PlaneCalibApp *app, const Eigen::Vector2i &imageSize)
{
	BaseWindow::init(app, imageSize);

	mMap = &mApp->getSystem().getMap();
	mTracker = &mApp->getSystem().getTracker();

	resize();

	mKeyBindings.addBinding(false, 't', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&ARWindow::toggleDisplayType), "Toggle display mode.");

	return true;
}

void ARWindow::GenerateARCubeVertices(std::vector<unsigned int> &triangleIndices, std::vector<Eigen::Vector4f> &vertices, std::vector<Eigen::Vector4f> &colors, std::vector<Eigen::Vector3f> &normals)
{
	const Eigen::Vector3f center = Eigen::Vector3f(0,0.25,-0.1f);
	const Eigen::Vector3f axis0 = Eigen::Vector3f(0.1, 0, 0);
	const Eigen::Vector3f axis1 = Eigen::Vector3f(0, 0.1, 0);
	const Eigen::Vector3f axis2 = Eigen::Vector3f(0, 0, 0.1);

	const Eigen::Vector4f cubeCorners[] = { (center - axis0 - axis1 - axis2).homogeneous(),	//0 ---
		(center + axis0 - axis1 - axis2).homogeneous(),		//1 +--
		(center + axis0 + axis1 - axis2).homogeneous(),		//2 ++-
		(center - axis0 + axis1 - axis2).homogeneous(),		//3 -+-
		(center - axis0 - axis1 + axis2).homogeneous(),		//4 --+
		(center + axis0 - axis1 + axis2).homogeneous(),		//5 +-+
		(center + axis0 + axis1 + axis2).homogeneous(),		//6 +++
		(center - axis0 + axis1 + axis2).homogeneous() }; 	//7 -++

	vertices.resize(6 * 4); //Six faces, four vertices per face
	colors.resize(6 * 4);
	normals.resize(6 * 4);
	triangleIndices.resize(6 * 6); //Six faces, six indices per face

	int faceIdx;

	//Face down
	faceIdx = 0;
	vertices[faceIdx * 4 + 0] = cubeCorners[0]; //---
	vertices[faceIdx * 4 + 1] = cubeCorners[1]; //+--
	vertices[faceIdx * 4 + 2] = cubeCorners[2]; //++-
	vertices[faceIdx * 4 + 3] = cubeCorners[3]; //-+-

	normals[faceIdx * 4 + 0] = normals[faceIdx * 4 + 1] = normals[faceIdx * 4 + 2] = normals[faceIdx * 4 + 3] = -axis2;

	//Face up
	faceIdx = 1;
	vertices[faceIdx * 4 + 0] = cubeCorners[4]; //--+
	vertices[faceIdx * 4 + 1] = cubeCorners[5]; //+-+
	vertices[faceIdx * 4 + 2] = cubeCorners[6]; //+++
	vertices[faceIdx * 4 + 3] = cubeCorners[7]; //-++

	normals[faceIdx * 4 + 0] = normals[faceIdx * 4 + 1] = normals[faceIdx * 4 + 2] = normals[faceIdx * 4 + 3] = axis2;

	//Face right
	faceIdx = 2;
	vertices[faceIdx * 4 + 0] = cubeCorners[5]; //+-+
	vertices[faceIdx * 4 + 1] = cubeCorners[6]; //+++
	vertices[faceIdx * 4 + 2] = cubeCorners[2]; //++-
	vertices[faceIdx * 4 + 3] = cubeCorners[1]; //+--

	normals[faceIdx * 4 + 0] = normals[faceIdx * 4 + 1] = normals[faceIdx * 4 + 2] = normals[faceIdx * 4 + 3] = axis0;

	//Face left
	faceIdx = 3;
	vertices[faceIdx * 4 + 0] = cubeCorners[4]; //--+
	vertices[faceIdx * 4 + 1] = cubeCorners[7]; //-++
	vertices[faceIdx * 4 + 2] = cubeCorners[3]; //-+-
	vertices[faceIdx * 4 + 3] = cubeCorners[0]; //---

	normals[faceIdx * 4 + 0] = normals[faceIdx * 4 + 1] = normals[faceIdx * 4 + 2] = normals[faceIdx * 4 + 3] = -axis0;

	//Face front
	faceIdx = 4;
	vertices[faceIdx * 4 + 0] = cubeCorners[7]; //-++
	vertices[faceIdx * 4 + 1] = cubeCorners[6]; //+++
	vertices[faceIdx * 4 + 2] = cubeCorners[2]; //++-
	vertices[faceIdx * 4 + 3] = cubeCorners[3]; //-+-

	normals[faceIdx * 4 + 0] = normals[faceIdx * 4 + 1] = normals[faceIdx * 4 + 2] = normals[faceIdx * 4 + 3] = axis1;

	//Face back
	faceIdx = 5;
	vertices[faceIdx * 4 + 0] = cubeCorners[5]; //+-+
	vertices[faceIdx * 4 + 1] = cubeCorners[4]; //--+
	vertices[faceIdx * 4 + 2] = cubeCorners[0]; //---
	vertices[faceIdx * 4 + 3] = cubeCorners[1]; //+--

	normals[faceIdx * 4 + 0] = normals[faceIdx * 4 + 1] = normals[faceIdx * 4 + 2] = normals[faceIdx * 4 + 3] = -axis1;

	//Other properties
	for (faceIdx = 0; faceIdx<6; faceIdx++)
	{
		//Set color
		colors[faceIdx * 4 + 0] = colors[faceIdx * 4 + 1] = colors[faceIdx * 4 + 2] = colors[faceIdx * 4 + 3] = StaticColors::Green(1, 0.5f + 0.5f / 5 * faceIdx);

		//Set indices
		triangleIndices[faceIdx * 6 + 0] = faceIdx * 4 + 0;
		triangleIndices[faceIdx * 6 + 1] = faceIdx * 4 + 1;
		triangleIndices[faceIdx * 6 + 2] = faceIdx * 4 + 2;
		triangleIndices[faceIdx * 6 + 3] = faceIdx * 4 + 0;
		triangleIndices[faceIdx * 6 + 4] = faceIdx * 4 + 2;
		triangleIndices[faceIdx * 6 + 5] = faceIdx * 4 + 3;
	}
}

void ARWindow::updateState()
{
	shared_lock<shared_mutex> lockRead(mApp->getSystem().getMap().getMutex());

	//Clear all
	mFeatureVertices.clear();
	mFeatureColors.clear();
	mImagePoints.clear();
	mImagePointColors.clear();

	//Add features
	mTrackerCamera = &mApp->getSystem().getCamera();
	mTrackerPoseR = mTracker->mCurrentPoseR;
	mTrackerPoseT = mTracker->mCurrentPoseT;

	const float pointAlpha = 0.6f;

	if (mDisplayType == EDisplayType::ShowMatches)
	{
		//Get matches from tracker
		for (auto &match : mTracker->getMatches())
		{
			Eigen::Vector4f color = StaticColors::Blue(pointAlpha);

			//Position is in 3D world coordinates
			mFeatureVertices.push_back(match.getFeature().mPosition3D.homogeneous());
			mFeatureColors.push_back(color);
		}
	}
	else
	{
		//Determine features in view
		//Add 3D features
		for(auto &featurePtr : mApp->getSystem().getMap().getFeatures())
		{
			const Feature &feature = *featurePtr;

			//Color
			Eigen::Vector4f color;
			color = StaticColors::Blue(pointAlpha);

			//Add
			mFeatureVertices.push_back(feature.mPosition3D.homogeneous());
			mFeatureColors.push_back(color);
		}
	}

	//Cube
	mCubeTriangleIndices.clear();
	mCubeVertices.clear();
	mCubeColors.clear();
	mCubeNormals.clear();

	GenerateARCubeVertices(mCubeTriangleIndices, mCubeVertices, mCubeColors, mCubeNormals);
}

void ARWindow::resize()
{
	mTiler.configDevice(Eigen::Vector2i::Zero(), UserInterfaceInfo::Instance().getScreenSize(), 1);
	mTiler.fillTiles();
	mTiler.setImageMVP(0, mImageSize);
}

void ARWindow::draw()
{
    mTiler.setActiveTile(0);
    mShaders->getTexture().setMVPMatrix(mTiler.getMVP());
	mShaders->getColor().setMVPMatrix(mTiler.getMVP());

	mShaders->getColorCamera().setMVPMatrix(mTiler.getMVP());
	mShaders->getColorCamera().setCamera(*mTrackerCamera);
	mShaders->getColorCamera().setPose(mTrackerPoseR, mTrackerPoseT);

	//Draw texture
	mShaders->getTexture().renderTexture(mCurrentImageTextureTarget, mCurrentImageTextureId, mImageSize, 1.0f);

	//Draw image points
	glPointSize(8);
	mShaders->getColor().drawVertices(GL_POINTS, mImagePoints.data(), mImagePointColors.data(), mImagePoints.size());

	//Draw features
	glPointSize(8);
	mShaders->getColorCamera().drawVertices(GL_POINTS, mFeatureVertices.data(), mFeatureColors.data(), mFeatureVertices.size());

	//Draw AR cube
    if(!mCubeTriangleIndices.empty())
    {
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LESS);

		mShaders->getColorCamera().drawVertices(GL_TRIANGLES, mCubeTriangleIndices.data(), mCubeTriangleIndices.size(), mCubeVertices.data(), mCubeColors.data());

	    glDisable(GL_DEPTH_TEST);
    }
}

void ARWindow::toggleDisplayType()
{
	shared_lock_guard<shared_mutex> lock(mApp->getSystem().getMap().getMutex());

	switch (mDisplayType)
	{
	case EDisplayType::ShowMatches: mDisplayType = EDisplayType::ShowStableFeatures; break;
	case EDisplayType::ShowStableFeatures: mDisplayType = EDisplayType::ShowAllFeatures; break;
	case EDisplayType::ShowAllFeatures: mDisplayType = EDisplayType::ShowMatches; break;
	}
	updateState();
}

} /* namespace dtslam */
