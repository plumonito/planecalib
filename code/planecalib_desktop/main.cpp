//Dummy file
#include <memory>
#include <iostream>
#include "planecalib_ui/PlaneCalibApp.h"

#include <planecalib_ui/UserInterfaceInfo.h>
#include <GL/glew.h>
#include <GL/freeglut.h>

#undef GFLAGS_DLL_DEFINE_FLAG
#define GFLAGS_DLL_DEFINE_FLAG
#include <gflags/gflags.h>

#include <ceres/ceres.h>

namespace planecalib
{
///////////////////////////////////////////////////////

DEFINE_int32(PyramidMaxTopLevelWidth, 320, "Maximum width of the highest pyramid level for a frame.");
DEFINE_int32(SBIMaxWidth, 60, "Maximum width for the Small Blurry Image, input will be downsampled until width is less than this.");
DEFINE_int32(FeatureDetectorThreshold, 10, "Threshold for the keypoint detector");
DEFINE_int32(MatcherPixelSearchDistance, 8, "The search distance for matching features (distance from point projection or from epiplar line). Units in pixels of the highest pyramid level.");


DEFINE_int32(DriverCameraId, 0, "Id of the camera to open (OpenCV).");
DEFINE_string(DriverDataPath, "c:/code/dslam/datasets", "Path to all data files (videos, camera calibrations, etc.)");
DEFINE_string(DriverVideoFile, "", "Name of the video file to use (e.g. rotation3.mp4). If both VideoFile and SequenceFormat are empty, the camera is used.");
DEFINE_string(DriverSequenceFormat, "", "sprintf format for the sequence (e.g. \"/cityOfSights/CS_BirdsView_L0/Frame_%.5d.jpg\". This is appended to the data path. If both VideoFile and SequenceFormat are empty, the camera is used.");
DEFINE_int32(DriverSequenceStartIdx, 0, "Start index for the image sequence.");
DEFINE_int32(DriverDropFrames, 0, "The system will ignore this many frames per iteration, effectively lowering the frame rate or skipping frames in a video.");
DEFINE_int32(DriverMaxImageWidth, 960, "Maximum width of input image. Input will be downsampled to be under this width.");
DEFINE_bool(DriverSingleThreaded, false, "Use a single thread for easier debugging.");
DEFINE_string(DriverRecordPath, "record/", "Path where the frames will be stored in case of recording.");

///////////////////////////////////////////////////////

DEFINE_int32(WindowWidth, 640, "Initial width of the window.");
DEFINE_int32(WindowHeight, 480, "Initial height of the window.");

}


planecalib::PlaneCalibApp *gApp = NULL;

int gWindowId;

void changeSize(int w, int h)
{
	planecalib::UserInterfaceInfo::Instance().setScreenSize(Eigen::Vector2i(w, h));
	gApp->resize();
}


void renderScene(void)
{
	if(gApp->getFinished())
	{
		glutDestroyWindow(gWindowId);
		exit(0);
	}

	gApp->draw();
	glutSwapBuffers();
}

void pressKey(unsigned char key, int x, int y)
{
	planecalib::UserInterfaceInfo::Instance().setKeyState(key, true);
	gApp->keyDown(false, key);
}

void releaseKey(unsigned char key, int x, int y)
{
	planecalib::UserInterfaceInfo::Instance().setKeyState(key, false);
	gApp->keyUp(false, key);
}

void pressSpecial(int key, int x, int y)
{
	planecalib::UserInterfaceInfo::Instance().setSpecialKeyState(key, true);
	gApp->keyDown(true, key);
}

void releaseSpecial(int key, int x, int y)
{
	planecalib::UserInterfaceInfo::Instance().setSpecialKeyState(key, false);
	gApp->keyUp(true, key);
}

void mouseEvent(int id, int state, int x, int y)
{
	if(state==GLUT_DOWN)
		gApp->touchDown(id, x, y);
	else if(state == GLUT_UP)
		gApp->touchUp(id, x, y);
}

void mouseMoveEvent(int x, int y)
{
	gApp->touchMove(x, y);
}

void __declspec(dllexport) dummy()
{
	ceres::Solver::Options options;
	ceres::Solver::Summary s;
	ceres::Problem p;
	ceres::Solve(options, &p, &s);
}

int main(int argc, char**argv)
{
	google::ParseCommandLineFlags(&argc, &argv, true);

	//cv::Size initialSize(1980,1040);
	Eigen::Vector2i initialSize(planecalib::FLAGS_WindowWidth, planecalib::FLAGS_WindowHeight);

	//init GLUT and create window
	glutInit(&argc, argv );
	glutInitDisplayMode(GLUT_DEPTH | GLUT_DOUBLE | GLUT_RGBA | GLUT_ALPHA);
	//glutInitWindowPosition(900,10);
	glutInitWindowSize(initialSize.x(),initialSize.y());
	planecalib::UserInterfaceInfo::Instance().setScreenSize(initialSize);
	gWindowId = glutCreateWindow("dslam");

	//Glew
	if(glewInit() != GLEW_OK)
	{
		MYAPP_LOG << "Error initializing GLEW!\n";
		return 0;
	}

	// register callbacks
	glutDisplayFunc(renderScene);
	glutReshapeFunc(changeSize);
	glutIdleFunc(renderScene);

	glutIgnoreKeyRepeat(0);
	glutKeyboardFunc(pressKey);
	glutKeyboardUpFunc(releaseKey);
	glutSpecialFunc(pressSpecial);
	glutSpecialUpFunc(releaseSpecial);
	glutMouseFunc(mouseEvent);
	glutMotionFunc(mouseMoveEvent);

	gApp = new planecalib::PlaneCalibApp();

	if (!gApp->init())
	{
		delete gApp;
		return 1;
	}

	// enter GLUT event processing cycle
	glutMainLoop();

	delete gApp;

	return 0;
}
