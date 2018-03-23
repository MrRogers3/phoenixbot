/*
 * aruco.cpp
 *
 *  Created on: Apr 4, 2017
 *      Author: Ryan Owens
 */

#include "aruco.h"
/*
 * Aruco
 *
 * Class for detecting Aruco markers and getting Pose Estimation
 *
 */
Aruco::Aruco(int dictionaryId) {
        this->dictionaryId = dictionaryId;
        // if dictioniary ID < 0 use custom dictionary file
        this->dictionaryId = dictionaryId;
        if (dictionaryId < 0) {
                this->getCustomDictionary();
        } else {
                this->dictionary = cv::aruco::getPredefinedDictionary(
                        cv::aruco::PREDEFINED_DICTIONARY_NAME(this->dictionaryId));
        } //set numbers of bits and markers from a pre-made dictionary.
        this->detectorParams = cv::aruco::DetectorParameters::create();
        //this->detectorParams->doCornerRefinement = true;
        if (!this->setCameraCalibration(
                    "$HOME/phoenixbot/logitech.yml")) {
                std::cout << "Could not Open the camera calibration file" << std::endl;
        }
}

/*
 * Aruco
 *
 * Class Destructor
 *
 */
Aruco::~Aruco() {
        this->camera.release();
}

/*
 * Aruco
 *
 * Class for detecting Aruco markers and getting Pose Estimation
 *
 */
cv::Mat Aruco::getFrame() {
        cv::Mat mt;
        if (!this->camera.isOpened()) {
                std::cout << "Camera is not opened, returning empty mat" << std::endl;
                return mt;
        }
        this->camera >> mt;
        // if (this->flipVertical)
        //   flip(mt, mt, 1);
        return mt;
}
/*
 * Set Camera Calibration
 *
 * Updates the Camera Distortion Matrixes from the file
 *
 */
bool Aruco::setCameraCalibration(std::string filename) {
        std::string fname = this->calibrationFilePath + filename;
        if (access(fname.c_str(), F_OK) == -1) //access(name,amode) return -1 if the path to the file does not exist. c_str() convert string into null terminated string (another format for string). amode is what kind of check I want from a pre-define list R_OK,W_OK,X_OK. F_OK check for existence of the path.
                return false;
        this->currentCalibrationFile = filename;
        return this->readCameraCalibration(filename);
}

/*
 * Read Camera Calibration
 *
 * Reads in the Camera Callibration for use with Pose Estimation
 *
 */
bool Aruco::readCameraCalibration(std::string filename) {
        cv::FileStorage fs(filename, cv::FileStorage::READ); //To read an existed XML,YAML file
        if (!fs.isOpened())
                return false;
        fs["camera_matrix"] >> this->cameraMatrix; //There is a matrix values under the name "camera_matrix". This line transfer those values into cameraMatrix.
        fs["distortion_coefficients"] >> this->distortionCoefficients; // ^^
        fs.release(); //Deconstruct the object.
        return true;
} //Example https://docs.opencv.org/2.4/modules/core/doc/xml_yaml_persistence.html

/*
 * Get Custom Dictionary
 *
 * Sets the Dictionary to a custom dictionary
 *
 */
bool Aruco::setDictionary(int dictionaryId) {
        if (dictionaryId < 0) {
                this->dictionaryId = dictionaryId;
                return getCustomDictionary();
        } else {
                this->dictionaryId = dictionaryId;
                this->dictionary = cv::aruco::getPredefinedDictionary(
                        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
                return true;
        }
} //Methode to change dictionary. Refer to Ln:19-24.
/*
 * Get Custom Dictionary
 *
 * Sets the Dictionary to a custom dictionary
 *
 */
bool Aruco::getCustomDictionary() {
        int markerSize, maxCorrectionBits;
        cv::Mat bytesList;
        cv::FileStorage fs(this->customDictionaryFile, cv::FileStorage::READ);
        if (!fs.isOpened())
                return false;
        fs["MarkerSize"] >> markerSize;
        fs["MaxCorrectionBits"] >> maxCorrectionBits;
        fs["BytesList"] >> bytesList;
        this->dictionary =
                new cv::aruco::Dictionary(bytesList, markerSize, maxCorrectionBits); //Creation of new dictionary.
        return true;
}

/*
 * Vector Contains
 *
 * Helper function to check if a particular integer Vector Contains
 * a value
 *
 */
bool Aruco::vectorContains(std::vector<int> vec, int val) { //std::vector<> works like an array but can change size dymanically.
        if (find(vec.begin(), vec.end(), val) != vec.end()) //find(first, last, val) check if the val exist within the given range [first,last). Will return the the first existed element that is the same as val. If not found, will retrun the last element.
                return true;
        return false;
}

/*
 * Get Pose
 *
 * Get the X, Y, and Z Rotation and Translation vectors for a
 * particular Aruco Marker in the View
 *
 * returns all zeros when not in View
 * returns as TX TY TZ RX RY RZ
 *
 */
std::vector<double> Aruco::getPose(int arucoId, cv::Mat *frame) {
        std::vector<double> rottransvec;

        // Camera Calibration Failed...No files?
        if ((cameraMatrix.empty() || cv::countNonZero(cameraMatrix) < 1) || //countNonZero(name) return number of non-zero elements in name.
            (distortionCoefficients.empty() ||
             cv::countNonZero(distortionCoefficients) < 1)) {
                std::cout << "Could not get the pose, no callibration" << std::endl;
                return rottransvec;
        }

        cv::Mat img = this->getFrame();

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejected; //Point2f consist of coordinate point of x and y. The "f" after Point2 is clarification that those coordinates are float values. "d" is double and "i" is int.
        std::vector<cv::Vec3d> rvecs, tvecs; //Look up cv::Vec3d. rvecs is rotational vector. tvecs is translational vector.

        // detect markers and estimate pose
        cv::aruco::detectMarkers(img, this->dictionary, corners, ids, detectorParams,
                                 rejected); //detectMarkers(...) reads from img, dictionary, and ids. Stores values in corners, detectorParams, and rejected. For more details: https://docs.opencv.org/3.4.0/d5/dae/tutorial_aruco_detection.html
        if (ids.size() > 0 && this->vectorContains(ids, arucoId)) {
                cv::aruco::estimatePoseSingleMarkers(corners, this->arucoSquareSize,
                                                     cameraMatrix, distortionCoefficients,
                                                     rvecs, tvecs); //estimate of what a marker would look like in camera. Params:(corners, markerLength, cameraMatrix, distortionCoefficients, rvecs, tvecs)

                size_t index = find(ids.begin(), ids.end(), arucoId) - ids.begin();

                cv::Vec3d translation = tvecs[index];
                cv::Vec3d rotation = rvecs[index];
                rottransvec.clear(); // Clear all of the zero
                for (size_t i = 0; i < translation.rows; i++) { //size_t is unsigned integer
                        rottransvec.push_back(translation[i]); //push_back(val) add val to the end of the vector.
                }

                for (size_t i = 0; i < rotation.rows; i++) {
                        rottransvec.push_back(rotation[i]);
                }
        }
        return rottransvec;
} //return rotational and translational vectors of a marker.

/*
 * Markers in View
 *
 * Get an array (vector) of all Aruco Markers in the current view
 *
 */
std::vector<int> Aruco::arucoMarkersInView(cv::Mat *frame) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejected;
        cv::Mat img;
        if (frame == nullptr) { //nullptr is a null for pointer
                // if (!this->m_camDevice->isOpen())
                if (!this->camera.isOpened())
                        if (!this->openCamera())
                                return ids;
                img = this->getFrame();
        } else {
                img = *frame;
        }

        cv::aruco::detectMarkers(img, this->dictionary, corners, ids, detectorParams,
                                 rejected);

        return ids;
} // return all ids currently in veiw of camera.

/*
 * Marker corners in view
 *
 */
std::vector<std::vector<cv::Point2f> >
Aruco::arucoMarkerCorners(cv::Mat *frame) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejected;

        cv::Mat img;
        if (frame == nullptr) {
                // if (!this->m_camDevice->isOpen())
                if (!this->camera.isOpened())
                        if (!this->openCamera())
                                return corners;
                img = this->getFrame();
        } else {
                img = *frame;
        }

        cv::aruco::detectMarkers(img, this->dictionary, corners, ids, detectorParams,
                                 rejected);

        return corners;
} //return all conrners of markers currently in view of camera.

/*
 * Marker In View
 *
 * Check if a particular Aruco Marker is in the current view
 *
 */
bool Aruco::arucoMarkerInView(int arucoId, cv::Mat *frame) {
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejected;
        cv::Mat img;
        if (frame == nullptr) {
                // if (!this->m_camDevice->isOpen())
                if (!this->camera.isOpened())
                        if (!this->openCamera())
                                return false;
                img = this->getFrame();
        } else {
                img = *frame;
        }
        cv::aruco::detectMarkers(img, this->dictionary, corners, ids, detectorParams,
                                 rejected);
        if (find(ids.begin(), ids.end(), arucoId) != ids.end())
                return true;
        return false;
}
//Number 10
/*
 * Calculate Chess Board Position
 *
 * Determines the chess board Position
 *
 */
void Aruco::calculateChessBoardPosition(std::vector<cv::Point3f> &corners) {
        for (size_t i = 0; i < (size_t) this->chessBoardDimensions.height; i++) {
                for (size_t j = 0; j < (size_t) this->chessBoardDimensions.width; j++) {
                        // X = j * square size, Y = i * square size, Z = 0.0
                        corners.push_back(cv::Point3f(j * this->chessBoardSquareSize,
                                                      i * this->chessBoardSquareSize, 0.0f)); //Point3f consist of coordinate points of x, y, and z. "f" means float. "d" is double and "i" is int.
                }
        }
}

/*
 * Calculate Chess Board Corners
 *
 * Determines all the chess board corner positions
 *
 */
void Aruco::calculateChessBoardCornersFromImages(
        std::vector<std::vector<cv::Point2f> > &foundCorners) {
        for (std::vector<cv::Mat>::iterator iter = this->images.begin();
             iter != this->images.end(); iter++) { //Works the same way as for-each in java but with more control.The std::vector<cv::Mat>::iterator iter makes iter an iterator for cv::Mat data type.
                std::vector<cv::Point2f> pointBuf;
                bool chessBoardFound = findChessboardCorners(
                        *iter, this->chessBoardDimensions, pointBuf, this->chessBoardFlags); //findChessboardCorners(image, patternSize, corners, flags) image is the source of what we are trying to find corners in. Pattern size contain info about number of inner corners per row and column. Corners is the output corners. Flag is the type of operation, there are 4 types. Look it up.
                if (chessBoardFound) {
                        foundCorners.push_back(pointBuf);
                }
        }
}

/*
 * Get Images From Camera
 *
 * Grab images from a camera and store for calibration
 *
 */

void Aruco::getImagesFromCamera() {
        cv::Mat frame, drawToFrame, oefficients,
                cameraMatrix = cv::Mat::eye(3, 3, CV_64F);
        std::vector<cv::Mat> savedImages;
        std::vector<std::vector<cv::Point2f> > markerCorners, rejectedCorners;

        // if (!this->m_camDevice->isOpen())
        if (!this->camera.isOpened())
                if (!this->openCamera())
                        return;
        usleep(5 * 1000000); //Put to sleep for x microseconds
        while (true) {
                // TODO Setup using LibWallaby Camera
                // if (!this->inputVideo.read(frame))
                //   break;

                std::vector<cv::Vec2f> foundPoints;
                bool found = false;
                found = findChessboardCorners(frame, this->chessBoardDimensions,
                                              foundPoints, this->chessBoardFlags);

                if (this->images.size() >= this->numImagesForCalibration)
                        return;
                if (found) {
                        cv::Mat temp;
                        frame.copyTo(temp); //Deep copy matrix from one to another.
                        this->images.push_back(temp);
                        // TODO Display to User to positon for next Frame...
                        usleep(5 * 1000);
                } else {
                        // TODO Display to User to reposition camera...
                }
        }
}

/*
 * Get Images From Camera
 *
 * Grab images from a camera and store for calibration
 *
 */
bool Aruco::calibrate(std::vector<cv::Mat> images) {
        this->images = images; // pass in images instead of get them here
        std::vector<std::vector<cv::Point2f> > imageSpacePoints;
        this->calculateChessBoardCornersFromImages(imageSpacePoints);
        std::vector<std::vector<cv::Point3f> > worldCornerPoints(1);
        this->calculateChessBoardPosition(worldCornerPoints[0]);
        worldCornerPoints.resize(imageSpacePoints.size(), worldCornerPoints[0]); //resize worldCornerPoints to be as big as the size of imageSpacePoints and fill the empy spots with copies of worldCornerPoints[0]

        std::vector<cv::Mat> rVecs, tVecs;
        this->distortionCoefficients = cv::Mat::zeros(8, 1, CV_64F);
        // Performs the Calibration
        calibrateCamera(worldCornerPoints, imageSpacePoints,
                        this->chessBoardDimensions, this->cameraMatrix,
                        this->distortionCoefficients, rVecs, tVecs); //It's long. Look it up.
        if (this->saveCalibration())
                return true;
        else
                return false;
}

/*
 * Save Calibration
 *
 * Save the custom calibration file
 *
 */
bool Aruco::saveCalibration() {
        std::string fname = "/cameraCalibration.yml";

        cv::FileStorage fs(fname, cv::FileStorage::WRITE);
        if (!fs.isOpened())
                return false;
        time_t tm; //time_t is a data type use to represent time.
        time(&tm); //Get current today time (in seconds) and stored it in tm.
        struct tm *t2 = localtime(&tm); //set t2 to stored current date. localtime(&tm) return date and the specified time of the day (tm).
	//Exmaple: http://www.cplusplus.com/reference/ctime/time/
        char buf[1024];
        strftime(buf, sizeof(buf), "%c", t2); // strtime(ptr, maxsize, format, timeptr) This function copies and stores calendar time of timeptr into ptr in specified format (look it up). Maxsize limits number of char that can be copy into ptr.

        fs << "calibration_time" << buf;
        fs << "camera_matrix" << this->cameraMatrix;
        fs << "distortion_coefficients" << this->distortionCoefficients;
        fs.release();
        this->setCameraCalibration(this->calibrationFilePath);
        return true;
}

/*
 * Set Chess Board Size
 *
 * Sets the size for the chess board squares
 *
 */
void Aruco::setChessBoardSize(float sizeInMeters) {
        if (sizeInMeters > 0.0 && sizeInMeters < 1.0) {
                this->chessBoardSquareSize = sizeInMeters;
        } else {
                this->chessBoardSquareSize = 0.0235f; // Default size if printed on 8.5x11
        }
}

/*
 * Set Aruco Marker Size
 *
 * Sets the size for aruco markers
 *
 */
void Aruco::setArucoMarkerSize(float sizeInMeters) {
        if (sizeInMeters > 0.0 && sizeInMeters < 1.0) {
                this->arucoSquareSize = sizeInMeters;
        } else {
                this->arucoSquareSize = 0.025f; // Default size if printed on 8.5x11
        }
}

/*
 * Open Camera
 *
 * Opens the Camera
 *
 */
bool Aruco::openCamera() {
        std::cout << "Opening Camera" << std::endl;
        if (this->camera.isOpened())
                return true;
        // Camera 1 is the default
        return this->camera.open(1);
}
