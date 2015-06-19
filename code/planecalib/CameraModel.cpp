#include "CameraModel.h"
#include "CameraModelCeres.h"

namespace planecalib {

//template class CameraModel_<NullCameraDistortionModel>;
//template class CameraModel_<RadialCameraDistortionModel>;
template class CameraModel_<DivisionDistortionModel>;


}
