#ifndef ARWINDOW_H_
#define ARWINDOW_H_

#include "planecalib/eutils.h"
#include "planecalib/CameraModel.h"
#include "BaseWindow.h"

namespace planecalib
{

class Map;
class PoseTracker;

class ARWindow: public BaseWindow
{
public:
	ARWindow():
		BaseWindow("ARWindow"), mDisplayType(EDisplayType::ShowMatches)
	{}

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	bool init(PlaneCalibApp *app, const Eigen::Vector2i &imageSize);
	void showHelp() const;

	void updateState();
    void resize();

//    void touchDown(int id, int x, int y);
//    void touchMove(int x, int y);
//    void touchUp(int id, int x, int y);

    void draw();

protected:
    ViewportTiler mTiler;

    Map *mMap;
    PoseTracker *mTracker;

	enum class EDisplayType
	{
		ShowMatches,
		ShowStableFeatures,
		ShowAllFeatures
	};
	EDisplayType mDisplayType;

    const CameraModel *mTrackerCamera;
	Eigen::Matrix3fr mTrackerPoseR;
	Eigen::Vector3f mTrackerPoseT;

	//Draw data
	std::vector<Eigen::Vector2f> mImagePoints;
	std::vector<Eigen::Vector4f> mImagePointColors;

	std::vector<Eigen::Vector4f> mFeatureVertices;
	std::vector<Eigen::Vector4f> mFeatureColors;

	std::vector<unsigned int> mCubeTriangleIndices;
	std::vector<Eigen::Vector4f> mCubeVertices;
	std::vector<Eigen::Vector4f> mCubeColors;
	std::vector<Eigen::Vector3f> mCubeNormals;

	void toggleDisplayType();
public:
	static void GenerateARCubeVertices(const Map &map, std::vector<unsigned int> &triangleIndices, std::vector<Eigen::Vector4f> &vertices, std::vector<Eigen::Vector4f> &colors, std::vector<Eigen::Vector3f> &normals);
};

} 

#endif /* ARWINDOW_H_ */
