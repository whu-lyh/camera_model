#ifndef CAMERAFACTORY_H
#define CAMERAFACTORY_H

#include <opencv2/core/core.hpp>

#include "camera_model/camera_models/Camera.h"

namespace camera_model
{
	class CameraFactory
	{
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
			CameraFactory();

		static std::shared_ptr<CameraFactory> instance(void);

		CameraPtr generateCamera(Camera::ModelType modelType,
			const std::string& cameraName,
			cv::Size imageSize) const;

		CameraPtr generateCameraFromYamlFile(const std::string& filename);

	private:
		static std::shared_ptr<CameraFactory> m_instance;
	};
}

#endif
