#ifndef MAPWINDOW_H_
#define MAPWINDOW_H_

#include <future>
#include <vector>
#include <array>
#include "planecalib/CameraDistortionModel.h"
#include "planecalib/eutils.h"
#include "BaseWindow.h"
#include "../shaders/StaticColors.h"

namespace planecalib {

struct MapWindow_DrawFeatureData
{
	Eigen::Vector4f center;
	Eigen::Vector4f solidColor;
};
struct MapWindow_DrawFrustumData
{
	std::vector<Eigen::Vector4f> frameVertices;
	std::vector<int> frameIndices;

	std::vector<Eigen::Vector4f> borderVertices;
	std::array<Eigen::Vector4f, 4> corners; //The four extreme corners of the frame
	Eigen::Vector4f center;

	Eigen::Vector4f color;

	unsigned int texTarget;
	unsigned int texId;
};

}

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(planecalib::MapWindow_DrawFeatureData);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(planecalib::MapWindow_DrawFrustumData);

namespace planecalib {

class Map;
class PoseTracker;
class Keyframe;
class Feature;

class MapWindow: public BaseWindow
{
public:
	MapWindow():
		BaseWindow("MapWindow"),
		mPointSize(4),
		mActiveDragType(EDragType::NoDragging)
	{
	}

	bool init(PlaneCalibApp *app, const Eigen::Vector2i &imageSize);
	void showHelp() const;

	void updateState();
    void resize();

    void touchDown(int id, int x, int y);
    void touchMove(int x, int y);
    void touchUp(int id, int x, int y);

    void draw();

protected:
    Map *mMap;
    PoseTracker *mTracker;

    float mPointSize;

	Eigen::Matrix3fr mSystemCameraK;
	Eigen::Matrix3fr mSystemCameraKinv;
	RadialCameraDistortionModel mSystemCameraDistortion;

    Eigen::Matrix3fr mViewerCameraK;
	Eigen::Matrix3fr mViewerCameraKinv;
	Eigen::Matrix3fr mViewerCameraR;
	Eigen::Vector3f mViewerCameraT;

	std::unordered_map<const Keyframe *, std::unique_ptr<TextureHelper>> mFrameTextures;

	//Dragging
	enum class EDragType
	{
		NoDragging=0,
		DraggingRotation,
		DraggignTranslation,
		DraggingCube
	};
	static const int kTranslateScale=20;
	EDragType mActiveDragType;
	Eigen::Matrix3fr mDragStartingR;
	Eigen::Vector3f mDragStartingT;
    Eigen::Vector2f mDragOrigin; //The origin of the dragging in pixel units
	Eigen::Vector2f mDragEnd; //The end of the dragging in pixel units (updates as the mouse moves even before the button is released)

    void increasePointSize() {mPointSize++;}
    void decreasePointSize() {mPointSize--; if(mPointSize<0) mPointSize=0;}

	void zoom(float ammount);
	void zoomIn() { zoom(+5.0f); }
	void zoomOut() {zoom(-5.0f);}

    void ensureValidRegion();
    void selectNextRegion();

    void performBA();
    std::future<void> mPerformBAFuture;

    static void PerformBATask(MapWindow *window);
    void performBATask();

    void forceNewKeyFrame();

    void startCube();
	void updateCube(const Eigen::Vector2f &origin, const Eigen::Vector2f &end);

    const TextureHelper &getFrameTexture(const Keyframe &frame);

    bool isFeatureMatched(const Feature &feature);

	//Drawing stuff
	float mMapDrawScale;

    //Drawing of feature
	typedef MapWindow_DrawFeatureData DrawFeatureData;
    std::vector<DrawFeatureData> mFeaturesToDraw;
    std::vector<DrawFeatureData> mMatchedFeaturesToDraw; //These are in a separate vector so they are drawn after and are always visible.
    void drawFeature(const DrawFeatureData &data);

	//Drawing of a camera frustum on the map
	typedef MapWindow_DrawFrustumData DrawFrustumData;
	std::vector<DrawFrustumData> mFrustumsToDraw;

	std::vector<Eigen::Vector4f> mPoseLog;

	DrawFrustumData prepareFrameFrustum(const Eigen::Matrix3fr &R, Eigen::Vector3f &t, unsigned int texTarget=0, unsigned int texID=0);
    void drawFrameFrustum(const DrawFrustumData &data);
};

} /* namespace dtslam */

#endif /* MAPWINDOW_H_ */
