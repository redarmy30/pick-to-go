
#include <iostream>  
#include<opencv2/core/core.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <ctime>



//#define __create__ 
//#define __find__
//#define __calibrate__
#define __getposition__
using namespace std;
namespace {
	const char* about = "Create an ArUco marker image";
	const char* keys =
		"{@outfile |<none> | Output image }"
		"{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
		"DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
		"DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
		"DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
		"{id       |       | Marker id in the dictionary }"
		"{ms       | 200   | Marker size in pixels }"
		"{bb       | 1     | Number of bits in marker borders }"
		"{si       | false | show generated image }";
}

double get_mean( double *arrray, int cs);

using namespace cv;
#ifdef __create__
int main(int argc, char *argv[]) {
	CommandLineParser parser(argc, argv, keys);
	parser.about(about);
	argc = 5;
	if (argc < 4) {
		parser.printMessage();
		printf("HERE!");
		return 0;
	}

	int dictionaryId = parser.get<int>("d");
	int markerId = parser.get<int>("id");
	int borderBits = parser.get<int>("bb");
	int markerSize = parser.get<int>("ms");
	bool showImage = parser.get<bool>("si");

	String out = parser.get<String>(0);



	Ptr<aruco::Dictionary> dictionary =
		aruco::getPredefinedDictionary(aruco::DICT_6X6_250);

	Mat markerImg;
	aruco::drawMarker(dictionary, markerId, markerSize, markerImg, borderBits);
	printf("HERE2!");

	imshow("marker", markerImg);
	waitKey(0);


	imwrite(out, markerImg);

	return 0;
}

#endif // __create__



#ifdef __find__

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["camera_matrix"] >> camMatrix;
	fs["distortion_coefficients"] >> distCoeffs;
	return true;
}



/**
*/
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
	FileStorage fs(filename, FileStorage::READ);
	if (!fs.isOpened())
		return false;
	fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
	fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
	fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
	fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
	fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
	fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
	fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
	fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
	fs["minDistanceToBorder"] >> params->minDistanceToBorder;
	fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
	fs["doCornerRefinement"] >> params->doCornerRefinement;
	fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
	fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
	fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
	fs["markerBorderBits"] >> params->markerBorderBits;
	fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
	fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
	fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
	fs["minOtsuStdDev"] >> params->minOtsuStdDev;
	fs["errorCorrectionRate"] >> params->errorCorrectionRate;
	return true;
}



/**
*/


int main(int argc, char *argv[]) {
	CommandLineParser parser(argc, argv, keys);
	parser.about(about);

	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	
	detectorParams->doCornerRefinement = true; // do corner refinement in markers

	

	String video;
	


	Ptr<aruco::Dictionary> dictionary =
		aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
	Mat camMatrix, distCoeffs;
	
	VideoCapture inputVideo(0);
	int waitTime;


	double totalTime = 0;
	int totalIterations = 0;

	while (inputVideo.grab()) {
		Mat image, imageCopy;
		inputVideo.read(image);

		double tick = (double)getTickCount();

		vector< int > ids;
		vector< vector< Point2f > > corners, rejected;
		vector< Vec3d > rvecs, tvecs;

		// detect markers and estimate pose
		aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
		
		int estimatePose = 0;
		float markerLength = 0.05;
		float showRejected = 0;

		if (estimatePose && ids.size() > 0)
			aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs,
				tvecs);

		double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
		totalTime += currentTime;
		totalIterations++;
		if (totalIterations % 30 == 0) {
			cout << "Detection Time = " << currentTime * 1000 << " ms "
				<< "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
		}

		// draw results
		image.copyTo(imageCopy);
		if (ids.size() > 0) {
			aruco::drawDetectedMarkers(imageCopy, corners, ids);

			if (estimatePose) {
				for (unsigned int i = 0; i < ids.size(); i++)
					aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
						markerLength * 0.5f);
			}
		}

		if (showRejected && rejected.size() > 0)
			aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

		imshow("out", imageCopy);
		char key = (char)waitKey(0);
		if (key == 27) break;
	}

	return 0;
}

#endif


#ifdef __calibrate__

static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
    fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
    fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
    fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
    fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
    fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
    fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
    fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
    fs["minDistanceToBorder"] >> params->minDistanceToBorder;
    fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
    fs["doCornerRefinement"] >> params->doCornerRefinement;
    fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
    fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
    fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
    fs["markerBorderBits"] >> params->markerBorderBits;
    fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
    fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
    fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
    fs["minOtsuStdDev"] >> params->minOtsuStdDev;
    fs["errorCorrectionRate"] >> params->errorCorrectionRate;
    return true;
}



/**
 */
static bool saveCameraParams(const string &filename, Size imageSize, float aspectRatio, int flags,
                             const Mat &cameraMatrix, const Mat &distCoeffs, double totalAvgErr) {
    FileStorage fs(filename, FileStorage::WRITE);
    if(!fs.isOpened())
        return false;

    time_t tt;
    time(&tt);
    struct tm *t2 = localtime(&tt);
    char buf[1024];
    strftime(buf, sizeof(buf) - 1, "%c", t2);

    fs << "calibration_time" << buf;

    fs << "image_width" << imageSize.width;
    fs << "image_height" << imageSize.height;

    if(flags & CALIB_FIX_ASPECT_RATIO) fs << "aspectRatio" << aspectRatio;

    if(flags != 0) {
        sprintf(buf, "flags: %s%s%s%s",
                flags & CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
                flags & CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
                flags & CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
                flags & CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "");
    }

    fs << "flags" << flags;

    fs << "camera_matrix" << cameraMatrix;
    fs << "distortion_coefficients" << distCoeffs;

    fs << "avg_reprojection_error" << totalAvgErr;

    return true;
}



/**
 */
int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);
	argc = 6;
    if(argc < 6) {
        parser.printMessage();
        return 0;
    }

//    int markersX = parser.get<int>("w");
	int markersX = 4;
//    int markersY = parser.get<int>("h");
	int markersY = 2;
//    float markerLength = parser.get<float>("l");
	float markerLength = 0.06;
//    float markerSeparation = parser.get<float>("s");
	float markerSeparation = 0.006;
//    int dictionaryId = parser.get<int>("d");
	int dictionaryId = 0;
//    string outputFile = parser.get<String>(0);
	string outputFile = "supertest";

    int calibrationFlags = 0;
    float aspectRatio = 1;
/*    if(parser.has("a")) {
        calibrationFlags |= CALIB_FIX_ASPECT_RATIO;
        aspectRatio = parser.get<float>("a");
    }
    if(parser.get<bool>("zt")) calibrationFlags |= CALIB_ZERO_TANGENT_DIST;
    if(parser.get<bool>("pc")) calibrationFlags |= CALIB_FIX_PRINCIPAL_POINT;
	*/
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    /*if(parser.has("dp")) {
        bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return 0;
        }
    }*/

    //bool refindStrategy = parser.get<bool>("rs");
	bool refindStrategy = 1;
    //int camId = parser.get<int>("ci");
	int camId = 0;
    String video;

    //if(parser.has("v")) {
//        video = parser.get<String>("v");
 //   }

   

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

    // create board object
	Mat markerImg1;
    Ptr<aruco::GridBoard> gridboard =
            aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
	gridboard->draw(Size(1700, 1200), markerImg1,5,1);
	imshow("marker", markerImg1);
    Ptr<aruco::Board> board = gridboard.staticCast<aruco::Board>();

    // collected frames for calibration
    vector< vector< vector< Point2f > > > allCorners;
    vector< vector< int > > allIds;
    Size imgSize;

    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;

        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        if(refindStrategy) aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) aruco::drawDetectedMarkers(imageCopy, corners, ids);
        putText(imageCopy, "Press 'c' to add current frame. 'ESC' to finish and calibrate",
                Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 0, 0), 2);

        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
        if(key == 'c' && ids.size() > 0) {
            cout << "Frame captured" << endl;
            allCorners.push_back(corners);
            allIds.push_back(ids);
            imgSize = image.size();
        }
    }

    if(allIds.size() < 1) {
        cerr << "Not enough captures for calibration" << endl;
        return 0;
    }

    Mat cameraMatrix, distCoeffs;
    vector< Mat > rvecs, tvecs;
    double repError;

    if(calibrationFlags & CALIB_FIX_ASPECT_RATIO) {
        cameraMatrix = Mat::eye(3, 3, CV_64F);
        cameraMatrix.at< double >(0, 0) = aspectRatio;
    }

    // prepare data for calibration
    vector< vector< Point2f > > allCornersConcatenated;
    vector< int > allIdsConcatenated;
    vector< int > markerCounterPerFrame;
    markerCounterPerFrame.reserve(allCorners.size());
    for(unsigned int i = 0; i < allCorners.size(); i++) {
        markerCounterPerFrame.push_back((int)allCorners[i].size());
        for(unsigned int j = 0; j < allCorners[i].size(); j++) {
            allCornersConcatenated.push_back(allCorners[i][j]);
            allIdsConcatenated.push_back(allIds[i][j]);
        }
    }
    // calibrate camera
    repError = aruco::calibrateCameraAruco(allCornersConcatenated, allIdsConcatenated,
                                           markerCounterPerFrame, board, imgSize, cameraMatrix,
                                           distCoeffs, rvecs, tvecs, calibrationFlags);

    bool saveOk = saveCameraParams(outputFile, imgSize, aspectRatio, calibrationFlags, cameraMatrix,
                                   distCoeffs, repError);

    if(!saveOk) {
        cerr << "Cannot save output file" << endl;
        return 0;
    } 

    cout << "Rep Error: " << repError << endl;
    cout << "Calibration saved to " << outputFile << endl;

    return 0;
}



#endif


#ifdef __getposition__

int main(int argc, char *argv[]) {


	VideoCapture inputVideo;
	inputVideo.open(0);
	//cv::Mat cameraMatrix, distCoeffs;
	//Mat cameraMatrix = (Mat1d(3, 3) << 6.7605173735173094e+02, 0., 3.7031077455138728e+02, 0., 7.1731191825575831e+02, 1.2136358605310052e+02, 0., 0., 1.);
	//Mat distCoeffs = (Mat1d(1, 5) << 2.0263252980560781e-01, -1.0355446957801111e+00, -5.8072849298604873e-02, 8.3655902978841262e-03, 1.5476950363624606e+00);
	Mat cameraMatrix = (Mat1d(3, 3) << 6.7605173735173094e+02, 0., 3.7031077455138728e+02, 0.,		7.1731191825575831e+02, 1.2136358605310052e+02, 0., 0., 1.);
	Mat distCoeffs = (Mat1d(1, 5) << 2.0263252980560781e-01, -1.0355446957801111e+00,		-5.8072849298604873e-02, 8.3655902978841262e-03,		1.5476950363624606e+00);
		// camera parameters are read from somewhere
	//readCameraParameters(cameraMatrix, distCoeffs);
	int markersX = 4;
	//    int markersY = parser.get<int>("h");
	int markersY = 2;
	//    float markerLength = parser.get<float>("l");
	float markerLength = 0.06;
	//    float markerSeparation = parser.get<float>("s");
	float markerSeparation = 0.006;
	//    int dictionaryId = parser.get<int>("d");
	int dictionaryId = 3;
	//    string outputFile = parser.get<String>(0);
	string outputFile = "supertest";
	double filtr[5];
	int i=0;
	int calibrationFlags = 0;
	float aspectRatio = 1;
	Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	Ptr<aruco::GridBoard> gridboard = aruco::GridBoard::create(markersX, markersY, markerLength, markerSeparation, dictionary);
	while (inputVideo.grab()) {
		cv::Mat image, imageCopy;
		inputVideo.retrieve(image);
		image.copyTo(imageCopy);
		std::vector<int> ids;
		std::vector<std::vector<cv::Point2f> > corners;
		cv::aruco::detectMarkers(image, dictionary, corners, ids);
		// if at least one marker detected
		if (ids.size() > 0) {
			cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
			cv::Vec3d rvec, tvec;
			int valid = aruco::estimatePoseBoard(corners, ids, gridboard, cameraMatrix, distCoeffs, rvec, tvec);
			/*for (int i = 0; i<ids.size(); i++) {
				cout << ids[i] << endl;
			}
			for (int i = 0; i<corners.size(); i++) {
				cout << corners[i] << endl;
			}*/

			// if at least one board marker detected
			if ((valid > 0) & (ids.size() >= 4)) {
				//try{ cv::aruco::drawAxis(imageCopy, cameraMatrix, distCoeffs, rvec, tvec, 0.2); }
				//catch (...) { cout << "default exception"; }
				cout << endl << endl;
				cout << "rotation:	" << rvec << endl << "translation:	" << tvec << endl << endl;

				filtr[i++] = tvec[2];
				i = i % 5;
				double mean = get_mean( filtr, 5);
				cout << filtr[i] / mean << "  mean" << endl;
				if ((tvec[2] > 0.5) & (filtr[i] / mean < 1.2) &(filtr[i] / mean > 0.8))  {
					if (tvec[0] > 0.2) cout << "rotate to right" << endl;
					if (tvec[0] < -0.2) cout << "rotate to left" << endl;
				}
			}
		}
		
		cv::imshow("out", imageCopy);
		char key = (char)cv::waitKey(10);
		if (key == 27)
			break;
	}
}



#endif // __getposition__




double get_mean(double* arrray, int sizze) {
	assert((sizze > 0, "size of array should be more than 0" ));
	double sum=0;
	for (int i = 0; i < sizze; ++i)
		sum += arrray[i];
	return (sum / sizze);
}
//	cv::Mat markerImage;
//	cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
//	cv::aruco::drawMarker(&dictionary, 23, 200, markerImage,1);


/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

License Agreement
For Open Source Computer Vision Library
(3-clause BSD License)

Copyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

* Neither the names of the copyright holders nor the names of the contributors
may be used to endorse or promote products derived from this software
without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/

	

