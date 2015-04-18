#include "MapWindow.h"
#include <thread>
#include <array>
#include "../shaders/Shaders.h"
#include "planecalib/PlaneCalibSystem.h"
#include "planecalib/Map.h"
#include "planecalib/PoseTracker.h"
#include "planecalib/HomographyCalibration.h"
#include "planecalib/flags.h"
#include "../flags.h"
#include "../PlaneCalibApp.h"

namespace planecalib {

void MapWindow::showHelp() const
{
	BaseWindow::showHelp();
	MYAPP_LOG << "Use left mouse button to rotate view and right mouse button to translate view.\n"
			<< "Green squares = 2D features\n"
			<< "Blue squares = 3D features\n"
			<< "Purple squares = temporary triangulations from tracker\n";
}

bool MapWindow::init(PlaneCalibApp *app, const Eigen::Vector2i &imageSize)
{
	BaseWindow::init(app, imageSize);
	
	mViewerCameraR = Eigen::Matrix3fr::Identity();
	mViewerCameraT = Eigen::Vector3f(0, 0, 5);

	resize();

	mKeyBindings.addBinding(false,'b',static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MapWindow::performBA),"Perform Bundle Adjustment on demand.");
	mKeyBindings.addBinding(false,'f',static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MapWindow::forceNewKeyFrame),"Force a new key frame to be added.");
	mKeyBindings.addBinding(false,'y',static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MapWindow::increasePointSize),"Increase GL point size.");
	mKeyBindings.addBinding(false,'Y',static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MapWindow::decreasePointSize),"Decrease GL point size.");
	mKeyBindings.addBinding(false, '-', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MapWindow::zoomIn), "Zoom in.");
	mKeyBindings.addBinding(false, '+', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MapWindow::zoomOut), "Zoom out.");
	mKeyBindings.addBinding(false, 'c', static_cast<KeyBindingHandler<BaseWindow>::SimpleBindingFunc>(&MapWindow::startCube), "Define cube surface.");
		
	return true;
}

void MapWindow::updateState()
{
	mMap = &mApp->getSystem().getMap();
	mTracker = &mApp->getSystem().getTracker();

	mSystemCameraK = mApp->getSystem().getCamera().getK();
	mSystemCameraKinv = mSystemCameraK.inverse();
	mSystemCameraDistortion = mApp->getSystem().getCamera().getDistortionModel();

	shared_lock<shared_mutex> lockRead(mMap->getMutex());

	/////////////////////////
    // Prepare draw objects
    mFeaturesToDraw.clear();
    mMatchedFeaturesToDraw.clear();
    mFrustumsToDraw.clear();

	//Pose log (not supported)
	mPoseLog.clear();

    //Features
	for(auto &featurePtr : mMap->getFeatures())
	{
		const Feature &feature = *featurePtr;

		DrawFeatureData data;
		bool isMatched = isFeatureMatched(feature);

		data.center = feature.mPosition3D.homogeneous();

		//Color
		//Check to see if feature has been matched
		const float alpha = 0.55f;
		if (isMatched)
			data.solidColor = StaticColors::Yellow(alpha);
		else
			data.solidColor = StaticColors::Blue(alpha);

		if(isMatched)
			mMatchedFeaturesToDraw.push_back(data);
		else
			mFeaturesToDraw.push_back(data);
	}

    //Key frames
    for(auto &framePtr : mMap->getKeyframes())
    {
		auto &frame = *framePtr;
    	//mFrustumsToDraw.push_back(prepareFrameFrustum((*it)->getPose(), (*it)->getCameraModel(), false));
    	auto &tex = getFrameTexture(frame);
    	mFrustumsToDraw.push_back(prepareFrameFrustum(framePtr->mPose3DR, frame.mPose3DT, tex.getTarget(), tex.getId()));
    }

    //Draw currently tracked frame
	mFrustumsToDraw.push_back(prepareFrameFrustum(mTracker->mCurrentPoseR, mTracker->mCurrentPoseT, mCurrentImageTextureTarget, mCurrentImageTextureId));
}

bool MapWindow::isFeatureMatched(const Feature &feature)
{
	auto match = mApp->getSystem().getTracker().getMatch(&feature);
	if(match)
		return true;
	return false;
}

void MapWindow::resize()
{
	Eigen::Vector2i screenSize = UserInterfaceInfo::Instance().getScreenSize();

	const float fx = 600;
	const float fy = 600;
	mViewerCameraK << fx, 0, (float)screenSize[0] / 2, 0, fy, (float)screenSize[1] / 2, 0,0,1;
	mViewerCameraKinv = mViewerCameraK.inverse();
}

void MapWindow::startCube()
{
	mActiveDragType = EDragType::DraggingCube;
}

void MapWindow::updateCube(const Eigen::Vector2f &origin, const Eigen::Vector2f &end)
{
	//std::vector<cv::Vec3f> points;

	//for(auto &feature : mFeaturesToDraw)
	//{
	//	cv::Point3f center(feature.center[0], feature.center[1], feature.center[2]);
	//	cv::Point2f screenPos = mViewerCamera.projectFromWorld(mViewerPose.apply(center));

	//	if(screenPos.x >= origin.x && screenPos.y >= origin.y && screenPos.x <= end.x && screenPos.y <= end.y)
	//		points.push_back(center);
	//}

	//if(points.size() < 3)
	//{
	//	mApp->disableARCube();
	//	return;
	//}

	//cv::Mat pointsMat(points.size(), 3, CV_32F, points.data());
	//cv::Matx33f covar;
	//cv::Matx13f mean;
	//cv::calcCovarMatrix(pointsMat, covar, mean, cv::COVAR_NORMAL | cv::COVAR_ROWS | cv::COVAR_SCALE, CV_32F);
	////DTSLAM_LOG << covar << "\n";

	//cv::Point3f pointsMean(mean(0,0), mean(0,1), mean(0,2));

	////Decompose
	//cv::SVD svd(covar, cv::SVD::MODIFY_A);
	////DTSLAM_LOG << svd.u << svd.w << svd.vt << "\n";
	//cv::Point3f axis1(svd.u.at<float>(0,0),svd.u.at<float>(1,0),svd.u.at<float>(2,0));
	//cv::Point3f axis2(svd.u.at<float>(0,1),svd.u.at<float>(1,1),svd.u.at<float>(2,1));
	//cv::Point3f axis3(svd.u.at<float>(0,2),svd.u.at<float>(1,2),svd.u.at<float>(2,2));

	//float maxEigenValue = svd.w.at<float>(0,0);
	//float cubeSize = 3*sqrtf(maxEigenValue);

	//axis1 *= cubeSize/2;
	//axis2 *= cubeSize/2;
	//axis3 *= cubeSize/2;

	////Decide normal direction
	//cv::Point3f cubeCenterA = pointsMean + axis3;
	//cv::Point3f cubeCenterB = pointsMean - axis3;

	//cv::Point3f cubeProjA = mViewerPose.apply(cubeCenterA);
	//cv::Point3f cubeProjB = mViewerPose.apply(cubeCenterB);

	//cv::Vec3f cubeCenter = (cubeProjA.z < cubeProjB.z) ? cubeCenterA : cubeCenterB;

	////Update
	//mApp->setARCube(cubeCenter, axis1, axis2, axis3);
}

void MapWindow::touchDown(int id, int x, int y)
{
	if(mActiveDragType == EDragType::NoDragging)
	{
		//No dragging active, start a new one
		switch(id)
		{
		case kMouseLeftButton:
		case kMouseRightButton:
			mActiveDragType = (id==kMouseLeftButton) ? EDragType::DraggingRotation : EDragType::DraggignTranslation;
			mDragOrigin = Eigen::Vector2f((float)x, (float)y);
			mDragStartingR = mViewerCameraR;
			mDragStartingT = mViewerCameraT;
			break;

		case kMouseScrollDown: zoomIn(); break;
		case kMouseScrollUp: zoomOut(); break;
		}
	}
	else if(mActiveDragType == EDragType::DraggingCube)
	{
		//Start the cube
		mDragOrigin = Eigen::Vector2f((float)x, (float)y);
	}
}

void MapWindow::zoom(float ammount)
{
	//Zoom
	Eigen::Vector3f disp = Eigen::Vector3f(0, 0, ammount / kTranslateScale);
	mViewerCameraT += disp;

	{
		shared_lock_guard<shared_mutex> lock(mApp->getSystem().getMap().getMutex());
		updateState();
	}
}

void MapWindow::touchMove(int x, int y)
{
	mDragEnd = Eigen::Vector2f((float)x, (float)y);
	if(mActiveDragType == EDragType::DraggingRotation)
	{
		//Rotation
		Eigen::Vector2f diff = mDragEnd - mDragOrigin;

		float angleY = diff[0] * M_PI / (UserInterfaceInfo::Instance().getScreenSize()[0] / 2);
		float angleX = diff[1] * M_PI / (UserInterfaceInfo::Instance().getScreenSize()[1] / 2);

		Eigen::Matrix3fr rotX = eutils::RotationX(angleX);
		Eigen::Matrix3fr rotY = eutils::RotationY(angleY);

		Eigen::Matrix3fr rotR = rotX * rotY;
		mViewerCameraR = rotR*mDragStartingR;
        {
        	shared_lock_guard<shared_mutex> lock(mApp->getSystem().getMap().getMutex());
        	updateState();
        }
	}
	else if(mActiveDragType == EDragType::DraggignTranslation)
	{
		//Translation
		Eigen::Vector3f diff;
		diff << (mDragEnd - mDragOrigin) / kTranslateScale, 0;
		mViewerCameraT = mDragStartingT + diff;

        {
			shared_lock_guard<shared_mutex> lock(mApp->getSystem().getMap().getMutex());
			updateState();
        }
	}
	else if(mActiveDragType == EDragType::DraggingCube)
	{
		//Update final cube
		updateCube(mDragOrigin, mDragEnd);
	}
}

void MapWindow::touchUp(int id, int x, int y)
{
	mActiveDragType = EDragType::NoDragging;
}

void MapWindow::draw()
{
	Eigen::Vector2i screenSize = UserInterfaceInfo::Instance().getScreenSize();

	Eigen::Matrix<float, 3, 4> viewerRt;
	viewerRt << mViewerCameraR, mViewerCameraT;

	Eigen::Matrix<float, 3, 4> KRt3 = mViewerCameraK * viewerRt;
	Eigen::Matrix4f viewerKRt;
	viewerKRt << KRt3, Eigen::RowVector4f(0,0,0,1);

	Eigen::Matrix4f mvp = ViewportTiler::GetImageSpaceMvp(screenSize, screenSize) * viewerKRt;
	mShaders->getColor().setMVPMatrix(mvp);
	mShaders->getTexture().setMVPMatrix(mvp);

	glPointSize(mPointSize);
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);

	//Draw pose log
	//mShaders->getColor().drawVertices(GL_LINE_STRIP, mPoseLog.data(), mPoseLog.size(), StaticColors::Red());
	std::vector<Eigen::Vector4f> testVertices;
	testVertices.push_back(Eigen::Vector4f(-1, -1, 0, 1));
	testVertices.push_back(Eigen::Vector4f(-1, 1, 0, 1));
	testVertices.push_back(Eigen::Vector4f(1, 1, 0, 1));
	testVertices.push_back(Eigen::Vector4f(1, -1, 0, 1));
	mShaders->getColor().drawVertices(GL_LINE_LOOP, testVertices.data(), testVertices.size(), StaticColors::Red());

    //Draw features
    for(auto &data : mFeaturesToDraw)
    {
    	drawFeature(data);
    }

    glDisable(GL_DEPTH_TEST);
    for(auto &data : mMatchedFeaturesToDraw)
    {
    	drawFeature(data);
    }
    glEnable(GL_DEPTH_TEST);

    //Draw frustums
    for(auto &frameData : mFrustumsToDraw)
    {
    	drawFrameFrustum(frameData);
    }

    //Draw cube
    //if(mApp->isARCubeValid())
    //{
    //	std::vector<unsigned int> triangleIndices;
    //	std::vector<cv::Vec4f> vertices;
    //	std::vector<cv::Vec4f> colors;
    //	std::vector<cv::Vec3f> normals;
    //    mApp->generateARCubeVertices(triangleIndices, vertices, colors, normals);

    //	mShaders->getColor().drawVertices(GL_TRIANGLES, triangleIndices.data(), triangleIndices.size(), vertices.data(), colors.data());
    //}

    glDisable(GL_DEPTH_TEST);

    //Draw current frame as reference
    //const int drawHeight = 100;
    //float imgAspect = (float)mImageSize.width / mImageSize.height;
    //cv::Size drawSize((int)(imgAspect*drawHeight),drawHeight);
    //mShaders->getTexture().setMVPMatrix(ViewportTiler::GetImageSpaceMvp(screenSize, screenSize));
    //mShaders->getTexture().renderTexture(mCurrentImageTextureTarget,mCurrentImageTextureId,drawSize,cv::Point2i(screenSize.width-drawSize.width, screenSize.height-drawSize.height));
}

void MapWindow::drawFeature(const DrawFeatureData &data)
{
	Eigen::Vector4f solidColor = data.solidColor;

	//Special color when dragging for cube
	if(mActiveDragType == EDragType::DraggingCube)
	{
		Eigen::Vector3f center(data.center[0], data.center[1], data.center[2]);
		Eigen::Vector2f screenPos = (mViewerCameraK*(mViewerCameraR*center + mViewerCameraT)).eval().hnormalized();

		if (screenPos[0] >= mDragOrigin[0] && screenPos[1] >= mDragOrigin[1] && screenPos[0] <= mDragEnd[0] && screenPos[1] <= mDragEnd[1])
			solidColor = StaticColors::White(1);
	}

	mShaders->getColor().drawVertices(GL_POINTS, &data.center, 1, solidColor);
}

const TextureHelper &MapWindow::getFrameTexture(const Keyframe &frame)
{
	auto it = mFrameTextures.find(&frame);
	if(it == mFrameTextures.end())
	{
		auto add = mFrameTextures.insert(std::make_pair(&frame, std::unique_ptr<TextureHelper>(new TextureHelper())));
		TextureHelper &texture = *add.first->second;
		texture.create(GL_RGB, frame.getColorImage().size());
		texture.update(frame.getColorImage());
		return texture;
	}
	else
		return *it->second;
}

MapWindow::DrawFrustumData MapWindow::prepareFrameFrustum(const Eigen::Matrix3fr &R, Eigen::Vector3f &t, unsigned int texTarget, unsigned int texID)
{
	DrawFrustumData data;
    const float kFrustumDepth = 0.1f*mMapDrawScale;

	data.texTarget = texTarget;
	data.texId = texID;
	data.color = StaticColors::Gray();

	//Center
	Eigen::Vector3f center = -R.transpose()*t;
	data.center = center.homogeneous();
	//Eigen::Vector3f vv = center + R.transpose()*Eigen::Vector3f(0,0,1);
	//data.frameVertices.push_back(vv.homogeneous());
	//data.frameVertices.push_back(data.center);
	//Corners
	std::array<Eigen::Vector2f, 4> cornersImg = {
		Eigen::Vector2f(0, 0), 
		Eigen::Vector2f(mImageSize[0], 0), 
		Eigen::Vector2f(mImageSize[0], mImageSize[1]),
		Eigen::Vector2f(0, mImageSize[1])
	};
	for (int i = 0; i < cornersImg.size(); i++)
	{
		Eigen::Vector2f xn = mSystemCameraDistortion.undistortPoint((mSystemCameraKinv*cornersImg[i].homogeneous()).eval().hnormalized());
		Eigen::Vector3f xc = R.transpose()*xn.homogeneous()*kFrustumDepth + center;
		data.corners[i] = xc.homogeneous();
	}

	//Border
	std::vector<Eigen::Vector2f> imgPoints;
	int x, y;
	const int kBorderStep = 10;
	for (x = 0, y = 0; x < mImageSize[0]; x += kBorderStep)
	{
		imgPoints.emplace_back(x,y);
	}
	for (x-=kBorderStep; y < mImageSize[1]; y+=kBorderStep)
	{
		imgPoints.emplace_back(x, y);
	}
	for (y-=kBorderStep; x > 0; x -= kBorderStep)
	{
		imgPoints.emplace_back(x, y);
	}
	for (x += kBorderStep; y > 0; y -= kBorderStep)
	{
		imgPoints.emplace_back(x, y);
	}
	for (int i = 0; i < imgPoints.size(); i++)
	{
		Eigen::Vector2f xn = mSystemCameraDistortion.undistortPoint((mSystemCameraKinv*imgPoints[i].homogeneous()).eval().hnormalized());
		Eigen::Vector3f xc = R.transpose()*xn.homogeneous()*kFrustumDepth + center;
		data.borderVertices.push_back( xc.homogeneous() );
	}

 //   data.tl = camera.unprojectToWorld(cv::Point2f(0, 0)) * kFrustumDepth;
	//data.tr = camera.unprojectToWorld(cv::Point2f((float)camera.getImageSize().width, 0)) * kFrustumDepth;
	//data.bl = camera.unprojectToWorld(cv::Point2f(0, (float)camera.getImageSize().height)) * kFrustumDepth;
	//data.br = camera.unprojectToWorld(cv::Point2f((float)camera.getImageSize().width, (float)camera.getImageSize().height)) * kFrustumDepth;

 //   data.center = -pose.getRotation().t() * pose.getTranslation();
 //   data.tl = pose.applyInv(data.tl);
 //   data.tr = pose.applyInv(data.tr);
 //   data.bl = pose.applyInv(data.bl);
 //   data.br = pose.applyInv(data.br);

	//if (regionId < 0)
	//else
	//    data.color = mRegionColors[regionId % mRegionColors.size()];

 
    return data;
}

void MapWindow::drawFrameFrustum(const DrawFrustumData &data)
{
	std::array<Eigen::Vector4f,5> colors;
	colors.fill(data.color);

	Eigen::Vector4f vertices[5] =
	{ data.corners[0], data.corners[1], data.corners[2], data.corners[3], data.center};
    unsigned int indices[8] =
    { 0, 4, 1, 4, 2, 4, 3, 4 };

    mShaders->getColor().drawVertices(GL_LINES, indices, 8, vertices, colors.data());
	//mShaders->getColor().drawVertices(GL_LINE_STRIP, data.frameVertices.data(), 2, StaticColors::Yellow());
	//mShaders->getColor().drawVertices(GL_LINE_STRIP, data.frameVertices.data(), 2, StaticColors::Yellow());
	mShaders->getColor().drawVertices(GL_LINE_LOOP, data.borderVertices.data(), data.borderVertices.size(), data.color);

 //   cv::Vec4f v2[] =
 //   { vertices[1], vertices[0], vertices[3], vertices[2] };

 //   glDepthMask(false);
 //   if (data.useTex)
 //   {
 //       cv::Vec2f textureCoords[] =
 //       { cv::Vec2f(1, 0), cv::Vec2f(0, 0), cv::Vec2f(1, 1), cv::Vec2f(0, 1) };

 //       mShaders->getTexture().renderTexture(GL_TRIANGLE_STRIP, data.texTarget, data.texId, v2, textureCoords, 4, 0.55f);
 //   }
 //   else
 //   {
 //       //Use this to show only solid gray
 //       colors.fill(StaticColors::Gray(0.55f,0.75f));
 //       mShaders->getColor().drawVertices(GL_TRIANGLE_STRIP, v2, colors.data(), 4);
 //   }
 //   glDepthMask(true);

}

void MapWindow::performBA()
{
	//mPerformBAFuture = std::async(std::launch::async, PerformBATask, this);
	performBATask();
}

void MapWindow::PerformBATask(MapWindow *window)
{
	window->performBATask();
}

void MapWindow::performBATask()
{
	//Profiler::Instance().setCurrentThreadName("mapBundleAdjuster");

	mApp->getSystem().doFullBA();
	{
		shared_lock_guard<shared_mutex> lock(mApp->getSystem().getMap().getMutex());
		updateState();
	}
}

void MapWindow::forceNewKeyFrame()
{
	//mSlam->getMapExpander().addKeyFrame();
}

} /* namespace dtslam */
