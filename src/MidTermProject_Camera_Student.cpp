/* INCLUDES FOR THIS PROJECT */
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "dataStructures.h"
#include "matching2D.hpp"

using namespace std;
using std::cout;
using std::endl;

/* MAIN PROGRAM */
int main(int argc, const char *argv[])
{

    /* INIT VARIABLES AND DATA STRUCTURES */

    // data location
    string dataPath = "../";

    // camera
    string imgBasePath = dataPath + "images/";
    string imgPrefix = "KITTI/2011_09_26/image_00/data/000000"; // left camera, color
    string imgFileType = ".png";
    int imgStartIndex = 0; // first file index to load (assumes Lidar and camera names have identical naming convention)
    int imgEndIndex = 9;   // last file index to load
    int imgFillWidth = 4;  // no. of digits which make up the file index (e.g. img-0001.png)

    // misc
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    const string detectorType = "ORB";
    // SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT
    const vector<string> descriptorTypeList{ "BRISK", "BRIEF", "ORB", "FREAK", "SIFT" };
    // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
    /** 
     * Some Invalid Combinations:
     * AKAZE/KAZE: only to itself
     * SIFT - ORB
     */

    const int num_epochs = 10;      // number of testing rounds

    vector<long long> num_kpts_list(descriptorTypeList.size(), 0);
    vector<long long> num_ROI_kpts_list(descriptorTypeList.size(), 0);
    vector<long long> num_matches_list(descriptorTypeList.size(), 0);
    vector<double> t_detection_list(descriptorTypeList.size(), 0.0);
    vector<double> t_extraction_list(descriptorTypeList.size(), 0.0);

    for (int i = 0; i < descriptorTypeList.size(); i++)
    {
        string descriptorType = descriptorTypeList[i];
    for (int j  = 0; j < num_epochs; ++j)
    {
    /* MAIN LOOP OVER ALL IMAGES */
    for (size_t imgIndex = 0; imgIndex <= imgEndIndex - imgStartIndex; imgIndex++)
    {
        /* LOAD IMAGE INTO BUFFER */

        // assemble filenames for current index
        ostringstream imgNumber;
        imgNumber << setfill('0') << setw(imgFillWidth) << imgStartIndex + imgIndex;
        string imgFullFilename = imgBasePath + imgPrefix + imgNumber.str() + imgFileType;

        // load image from file and convert to grayscale
        cv::Mat img, imgGray;
        img = cv::imread(imgFullFilename);
        cv::cvtColor(img, imgGray, cv::COLOR_BGR2GRAY);

        //// STUDENT ASSIGNMENT
        //// TASK MP.1 -> replace the following code with ring buffer of size dataBufferSize

        // push image into data frame buffer
        DataFrame frame;
        frame.cameraImg = imgGray;
        dataBuffer.push_back(frame);
        while (dataBuffer.size() > dataBufferSize) 
            dataBuffer.erase(dataBuffer.begin());
        // cout << "dataBuffer size is: " << dataBuffer.size() << endl;

        //// EOF STUDENT ASSIGNMENT
        cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;

        /* DETECT IMAGE KEYPOINTS */

        // extract 2D keypoints from current image
        vector<cv::KeyPoint> keypoints; // create empty feature list for current image
        // string detectorType = "SHITOMASI";

        //// STUDENT ASSIGNMENT
        //// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> SHITOMASI, HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0)
        {
            t_detection_list[i] += detKeypointsShiTomasi(keypoints, imgGray, false);
            num_kpts_list[i] += keypoints.size();
        }
        else if (detectorType.compare("HARRIS") == 0)
        {
            t_detection_list[i] += detKeypointsHarris(keypoints, imgGray, false);
            num_kpts_list[i] += keypoints.size();
        }
        else
        {
            t_detection_list[i] += detKeypointsModern(keypoints, imgGray, detectorType, false);
            num_kpts_list[i] += keypoints.size();
        }
        //// EOF STUDENT ASSIGNMENT

        //// STUDENT ASSIGNMENT
        //// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
        vector<cv::KeyPoint> ROI_keypoints;
        if (bFocusOnVehicle)
        {
            for (auto it = keypoints.begin(); it != keypoints.end(); it++)
            {
                if (vehicleRect.contains(cv::Point2i((int)it->pt.x, (int)it->pt.y)))
                {
                    ROI_keypoints.emplace_back(*it);
                }
            }
        }
        num_ROI_kpts_list[i] += ROI_keypoints.size();
        
        //// EOF STUDENT ASSIGNMENT

        // optional : limit number of keypoints (helpful for debugging and learning)
        // bool bLimitKpts = false;
        // if (bLimitKpts)
        // {
        //     int maxKeypoints = 50;

        //     if (detectorType.compare("SHITOMASI") == 0)
        //     { // there is no response info, so keep the first 50 as they are sorted in descending quality order
        //         ROI_keypoints.erase(ROI_keypoints.begin() + maxKeypoints, ROI_keypoints.end());
        //     }
        //     cv::KeyPointsFilter::retainBest(ROI_keypoints, maxKeypoints);
        //     cout << " NOTE: Keypoints have been limited!" << endl;
        // }

        // push keypoints and descriptor for current frame to end of data buffer
        (dataBuffer.end() - 1)->keypoints = ROI_keypoints;
        cout << "#2 : DETECT KEYPOINTS done" << endl;

        /* EXTRACT KEYPOINT DESCRIPTORS */

        //// STUDENT ASSIGNMENT
        //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        // string descriptorType = "BRISK"; // BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT
        t_extraction_list[i] += descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT

        // push descriptors for current frame to end of data buffer
        (dataBuffer.end() - 1)->descriptors = descriptors;

        cout << "#3 : EXTRACT DESCRIPTORS done" << endl;

        if (dataBuffer.size() > 1) // wait until at least two images have been processed
        {

            /* MATCH KEYPOINT DESCRIPTORS */

            vector<cv::DMatch> matches;
            string matcherType = "MAT_BF";        // MAT_BF, MAT_FLANN
            string descriptorBaseType = descriptorType.compare("SIFT") == 0? "DES_HOG" : "DES_BINARY"; // DES_BINARY, DES_HOG
            string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN

            //// STUDENT ASSIGNMENT
            //// TASK MP.5 -> add FLANN matching in file matching2D.cpp
            //// TASK MP.6 -> add KNN match selection and perform descriptor distance ratio filtering with t=0.8 in file matching2D.cpp

            num_matches_list[i] += matchDescriptors((dataBuffer.end() - 2)->keypoints, (dataBuffer.end() - 1)->keypoints,
                                (dataBuffer.end() - 2)->descriptors, (dataBuffer.end() - 1)->descriptors,
                                matches, descriptorBaseType, matcherType, selectorType);

            //// EOF STUDENT ASSIGNMENT

            // store matches in current data frame
            (dataBuffer.end() - 1)->kptMatches = matches;

            cout << "#4 : MATCH KEYPOINT DESCRIPTORS done" << endl;

            // visualize matches between current and previous image
            bVis = false;
            if (bVis)
            {
                cv::Mat matchImg = ((dataBuffer.end() - 1)->cameraImg).clone();
                cv::drawMatches((dataBuffer.end() - 2)->cameraImg, (dataBuffer.end() - 2)->keypoints,
                                (dataBuffer.end() - 1)->cameraImg, (dataBuffer.end() - 1)->keypoints,
                                matches, matchImg,
                                cv::Scalar::all(-1), cv::Scalar::all(-1),
                                vector<char>(), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

                string windowName = "Matching keypoints between two camera images";
                cv::namedWindow(windowName, 7);
                cv::imshow(windowName, matchImg);
                // cout << "Press key to continue to next image" << endl;
                // cv::waitKey(0); // wait for key to be pressed
            }
            bVis = false;
        }

    } // eof loop over all images

        dataBuffer.clear();

    } // end of epochs

        dataBuffer.clear();
        cout << descriptorTypeList[i] << " Done" << endl;
        cout << endl;

    } // end of looping over descriptors

    // Print Summary
    cout << endl;
    cout << "########### Descriptors Summary ###########" << endl;
    cout << "Over the " << num_epochs << " epochs on 10 images: " << endl;

    long long num_kpts = 0;         // number of detected keypoints 
    long long num_ROI_kpts = 0;     // number of keypoints in the ROI
    double t_detection = 0.0;       // average detection time
    for (int i = 0; i < descriptorTypeList.size(); i++)
    {
        // Calculate Avg Values for Detector
        t_detection += t_detection_list[i] / 10.0 / num_epochs;
        num_kpts += num_kpts_list[i] / 10 / num_epochs;
        num_ROI_kpts += num_ROI_kpts_list[i] / 10 / num_epochs;
        
        // Calculate Avg Values for Descriptor
        long long num_matched_kpts = num_matches_list[i] / 9 / num_epochs; // average number of matched keypoint
        double t_extraction = t_extraction_list[i] / 10.0 / num_epochs;       // average extraction time
        
        cout << "=========== " << descriptorTypeList[i] << " ===========" << endl;
        cout << "Avg Num. of Matched Keypoints: " << num_matched_kpts  << endl;
        cout << "Avg Extraction Time: " << t_extraction  << " ms" << endl;
    }

    cout << endl;
    cout << "############ Detector Summary ############" << endl;
    cout << "Using " << detectorType << ": " << endl;
    cout << "Avg Detection Time: " << t_detection / descriptorTypeList.size() << " ms" << endl;
    cout << "Avg Detected Keypoints: " << num_kpts / descriptorTypeList.size() << endl;
    cout << "Avg ROI Keypoints: " << num_ROI_kpts / descriptorTypeList.size() << endl;

    return 0;
}
