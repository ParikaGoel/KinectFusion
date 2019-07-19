#include <VirtualSensor.h>

bool VirtualSensor::init(const std::string &datasetDir, bool intrinsics){

	m_baseDir = datasetDir;

	// Read filename lists
	if (!readFileList(datasetDir + "depth.txt", m_filenameDepthImages, m_depthImagesTimeStamps)) return false;
	if (!readFileList(datasetDir + "rgb.txt", m_filenameColorImages, m_colorImagesTimeStamps)) return false;

	// Read tracking
	// if (!readTrajectoryFile(datasetDir + "groundtruth.txt", m_trajectory, m_trajectoryTimeStamps)) return false;

    if (m_filenameDepthImages.size() != m_filenameColorImages.size()) return false;

    if( intrinsics){
        // Image resolutions
        m_colorImageWidth = 640;
        m_colorImageHeight = 480;
        m_depthImageWidth = 640;
        m_depthImageHeight = 480;

        // Intrinsics
        m_colorIntrinsics << 525.0f, 0.0f, 319.5f,
                0.0f, 525.0f, 239.5f,
                0.0f, 0.0f, 1.0f;

        m_depthIntrinsics = m_colorIntrinsics;
    }
    m_colorExtrinsics.setIdentity();
    m_depthExtrinsics.setIdentity();

    m_depthFrame = new double[m_depthImageWidth * m_depthImageHeight];
    for (unsigned int i = 0; i < m_depthImageWidth * m_depthImageHeight; ++i) m_depthFrame[i] = 0.5f;

    m_colorFrame = new BYTE[4 * m_colorImageWidth * m_colorImageHeight];
    for (unsigned int i = 0; i < 4 * m_colorImageWidth * m_colorImageHeight; ++i) m_colorFrame[i] = 255;

    return true;
}

bool VirtualSensor::processNextFrame() {

    if (m_currentIdx == -1) m_currentIdx = 0;
    else m_currentIdx += m_increment;

    if ((unsigned int)m_currentIdx >= (unsigned int)m_filenameColorImages.size()) return false;

    std::cout << "ProcessNextFrame [" << m_currentIdx << " | " << m_filenameColorImages.size() << "]" << std::endl;

    FreeImageB rgbImage;
    rgbImage.LoadImageFromFile(m_baseDir + m_filenameColorImages[m_currentIdx]);
    memcpy(m_colorFrame, rgbImage.data, 4 * m_colorImageWidth * m_colorImageHeight);

    // depth images are scaled by 5000 (see https://vision.in.tum.de/data/datasets/rgbd-dataset/file_formats)
    FreeImageU16F dImage;
    dImage.LoadImageFromFile(m_baseDir + m_filenameDepthImages[m_currentIdx]);

    for (unsigned int i = 0; i < m_depthImageWidth * m_depthImageHeight; ++i) {
        if (dImage.data[i] == 0)
            m_depthFrame[i] = MINF;
        else
            m_depthFrame[i] = dImage.data[i] * 1.0f / 5000.0f;
    }

    return true;
}

unsigned int VirtualSensor::getCurrentFrameCnt() {
    return (unsigned int)m_currentIdx;
}

BYTE *VirtualSensor::getColorRGBX() {
    return m_colorFrame;
}

double *VirtualSensor::getDepth() {
    return m_depthFrame;

}
Eigen::Matrix3d VirtualSensor::getColorIntrinsics() {
    return m_colorIntrinsics;
}

Eigen::Matrix4d VirtualSensor::getColorExtrinsics() {
    return m_colorExtrinsics;
}

unsigned int VirtualSensor::getColorImageWidth() {
    return m_colorImageWidth;
}

unsigned int VirtualSensor::getColorImageHeight() {
    return m_colorImageHeight;
}

// depth (ir) camera info
Eigen::Matrix3d VirtualSensor::getDepthIntrinsics() {
    return m_depthIntrinsics;
}

Eigen::Matrix4d VirtualSensor::getDepthExtrinsics() {
    return m_depthExtrinsics;
}

unsigned int VirtualSensor::getDepthImageWidth() {
    return m_depthImageWidth;
}

unsigned int VirtualSensor::getDepthImageHeight() {
    return m_depthImageHeight;
}

// get current trajectory transformation
Eigen::Matrix4d VirtualSensor::getTrajectory() {
    return m_currentTrajectory;
}

bool VirtualSensor::readFileList(const std::string &filename, std::vector<std::string> &result,
                                 std::vector<double> &timestamps) {

    std::ifstream fileDepthList(filename, std::ios::in);
    if (!fileDepthList.is_open()) return false;
    result.clear();
    timestamps.clear();
    std::string dump;
    std::getline(fileDepthList, dump);
    std::getline(fileDepthList, dump);
    std::getline(fileDepthList, dump);
    while (fileDepthList.good()) {
        double timestamp;
        fileDepthList >> timestamp;
        std::string filename;
        fileDepthList >> filename;
        if (filename == "") break;
        timestamps.push_back(timestamp);
        result.push_back(filename);
    }
    fileDepthList.close();
    return true;
}

bool VirtualSensor::readTrajectoryFile(const std::string &filename, std::vector<Eigen::Matrix4d> &result,
                                       std::vector<double> &timestamps) {

    std::ifstream file(filename, std::ios::in);
    if (!file.is_open()) return false;
    result.clear();
    std::string dump;
    std::getline(file, dump);
    std::getline(file, dump);

    std::getline(file, dump);

    while (file.good()) {
        double timestamp;
        file >> timestamp;
        Eigen::Vector3d translation;
        file >> translation.x() >> translation.y() >> translation.z();
        Eigen::Quaterniond rot;
        file >> rot;

        Eigen::Matrix4d transf;
        transf.setIdentity();
        transf.block<3, 3>(0, 0) = rot.toRotationMatrix();
        transf.block<3, 1>(0, 3) = translation;

        if (rot.norm() == 0) break;

        transf = transf.inverse().eval();

        timestamps.push_back(timestamp);
        result.push_back(transf);
    }
    file.close();
    return true;
}

