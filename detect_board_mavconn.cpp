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


#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <iostream>
#include <chrono>

#include <time.h>
#include <mavconn/interface.h>

#include "mavconn_msg.h"

#define MAV 1

using namespace std;
using namespace cv;

using namespace mavconn;
using namespace mavlink;

using namespace std::chrono;

namespace {
const char* about = "Pose estimation using a ArUco Planar Grid board";
const char* keys  =
        "{lp       |       | File with launch parameters }";
}

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;

    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;

    return true;
}

/**
 */
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
    fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
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
static bool readMarkerLayout(string filename, vector<vector<Point3f>> &objPoints, vector<int> &ids) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;

    float mm_px = fs["mm_per_unit"];
    float axisLength = 0.15f * mm_px;
    mm_px *= 0.001;

    // Parse corners
    FileNode corners_node = fs["corners"];
    FileNodeIterator it = corners_node.begin(), it_end = corners_node.end();
    int idx = 0;
    vector<vector<float>> lbpval;
    
    for( ; it != it_end; ++it, idx++ )
    {
        // cout << "Reaning corner #" << idx << ": ";
        (*it) >> lbpval;
        //cout << lbpval[0][0] << endl;
        vector<Point3f> points;
        points.push_back(Point3f(mm_px*lbpval[0][0], mm_px*lbpval[0][1], mm_px*lbpval[0][2]));
        points.push_back(Point3f(mm_px*lbpval[1][0], mm_px*lbpval[1][1], mm_px*lbpval[1][2]));
        points.push_back(Point3f(mm_px*lbpval[2][0], mm_px*lbpval[2][1], mm_px*lbpval[2][2]));
        points.push_back(Point3f(mm_px*lbpval[3][0], mm_px*lbpval[3][1], mm_px*lbpval[3][2]));
        objPoints.push_back(points);
    }

    // Parse ids
    fs["ids"]  >> ids;

    //fs.release();

    return true;
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Matx33d &R)
{
    Matx33d Rt;
    transpose(R, Rt);
    Matx33d shouldBeIdentity = Rt * R;
    Matx33d I = Matx33d::eye();

    return  norm(I, shouldBeIdentity) < 2e-6;
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3d rotationMatrixToEulerAngles(Matx33d &R)
{
    // assert(isRotationMatrix(R));

    float sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return Vec3d(x, y, z);
}

/**
 */
int main(int argc, char *argv[]) {
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);

    if (argc < 2) {
        parser.printMessage();
        return 0;
    }

    String filename = parser.get<string>("lp");
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened()) {
        cerr << "Invalid launch file" << endl;
        return 0;
    }

    int dictionaryId = fs["dictionary"];
    bool showRejected = (int)fs["show_rejected"] == 1;
    bool refinedStrategy = (int)fs["refined_strategy"] == 1;
    int camId = fs["camera_id"];

    float camera_offset_x = fs["offset_x"];
    float camera_offset_y = fs["offset_y"];
    float camera_offset_z = fs["offset_z"];

    String video = fs["video_file"];
    String pipe = fs["video_pipeline"];
    String mavurl = fs["mavconn_url"];
    int visiongps = fs["vision_gps_msg"];
    int pose_msg_rate = fs["pose_msg_rate"];
    bool show_image = (int)fs["show_image"] == 1;

    Mat camMatrix, distCoeffs;
    bool readOk = readCameraParameters(fs["camera_param"], camMatrix, distCoeffs);
    if (!readOk) {
        cerr << "Invalid camera file" << endl;
        return 0;
    }

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    readOk = readDetectorParameters(fs["detector_param"], detectorParams);
    if (!readOk) {
        cerr << "Invalid detector parameters file" << endl;
        return 0;
    }
    detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX; // do corner refinement in markers

    vector<vector<Point3f>> objPoints_;
    vector<int> ids_;
    readOk = readMarkerLayout(fs["board_layout"], objPoints_, ids_);
    if (!readOk) {
        cerr << "Invalid layout parameters file" << endl;
        return 0;
    }

    if (!parser.check()) {
        parser.printErrors();
        return 0;
    }

    // Create a board
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    Ptr<aruco::Board> board = aruco::Board::create(objPoints_, dictionary, ids_);

    // Show board
    Ptr<aruco::GridBoard> gridboard = board.staticCast<aruco::GridBoard>();
    Mat boardImage;
    gridboard->draw( Size(640, 480), boardImage, 10, 1 );
    //imshow("board", boardImage);

    // Video Capture
    VideoCapture inputVideo;
    int waitTime;
    
    if (!pipe.empty()) {
        inputVideo.open(pipe);
        waitTime = 5;
    } else {
        inputVideo.open(camId);
	    inputVideo.set(CAP_PROP_FRAME_WIDTH, 640);
	    inputVideo.set(CAP_PROP_FRAME_HEIGHT, 480);
        inputVideo.set(CAP_PROP_FPS, 30);
        waitTime = 5;
    }

#ifdef MAV
    // Mavlink Interface
    MAVConnInterface::Ptr client;
    cout << mavurl << endl;
	// Mavlink connection from url
    client = MAVConnInterface::open_url(mavurl, 1, 240);

    // Mavlink message receive callback
	client->message_received_cb = [&](const mavlink_message_t * msg, const Framing framing) {
		int msgid = int(msg->msgid);

		if (msgid == mavlink::common::msg::STATUSTEXT::MSG_ID) {
			mavlink::common::msg::STATUSTEXT stt {};
			mavlink::MsgMap map(msg);
			
			stt.deserialize(map);
			cout << "STATUSTEXT: " << to_string(stt.text) << endl;
		} else if (msgid == mavlink::common::msg::TIMESYNC::MSG_ID) {
            mavlink::common::msg::TIMESYNC tms {};
            mavlink::MsgMap map(msg);

            tms.deserialize(map);

            uint64_t now_ns = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();

            // TODO: timesync handling
            if (tms.tc1 == 0) {
                send_timesync(client.get(), now_ns, tms.ts1);
                //cout << "TIMESYNC tc: " << to_string(tms.tc1) <<  " ts: " << to_string(tms.ts1) << endl;
            } else if (tms.tc1 > 0) {
                // Time offset between this system and the remote system is calculated assuming RTT for
                // the timesync packet is roughly equal both ways.
                // add_timesync_observation((tms.ts1 + now_ns - tms.tc1 * 2) / 2, tms.ts1, tms.tc1);
                //cout << "TIMESYNC offset: " << to_string((tms.ts1 + now_ns - tms.tc1) / 2) << " tc: " << to_string(tms.tc1) << " ts: " << to_string(tms.ts1) << endl;
                //cout << "TIMESYNC RTT: " << to_string(now_ns - tms.ts1) << " tc: " << to_string(tms.tc1) << " ts: " << to_string(tms.ts1) << endl;
            }
        }
	};
#endif // MAV

    bool first = true;

    u_int32_t reset_counter = 1;
    u_int64_t now_micros = 0;
    u_int64_t now_nanos = 0;
    double now, prev_send_pos, prev_heartbeat;
    double prev_iter = 0.0;
    double dt = 0.0;

    auto now_epoch = system_clock::now().time_since_epoch();
    now_micros = duration_cast<microseconds>(now_epoch).count();
    now = (double)now_micros / 1000000.0;
    prev_send_pos = (double)now_micros / 1000000.0;
    prev_heartbeat = (double)now_micros / 1000000.0;

    double pose_msg_period = 1.0 / (double)pose_msg_rate;

    Vec3d rvec = Vec3d(0.0, 0.0, 0.0);
    Vec3d tvec = Vec3d(0.0, 0.0, 0.0);
    Vec3d rvec_inv = Vec3d(0.0, 0.0, 0.0);
    Vec3d tvec_inv = Vec3d(0.0, 0.0, 0.0);
    Vec3d rot_vec = Vec3d(0.0, 0.0, 0.0);
    Vec3d tra_vec = Vec3d(0.0, 0.0, 0.0);
    Vec3d rvec_inv_prev = Vec3d(0.0, 0.0, 0.0);
    Vec3d tvec_inv_prev = Vec3d(0.0, 0.0, 0.0);
    Vec3d tvec_vel = Vec3d(0.0, 0.0, 0.0);
    Vec3d tvec_vel_prev = Vec3d(0.0, 0.0, 0.0);
    Vec3d filt_vel = Vec3d(0.0, 0.0, 0.0);

    bool poseEstimated = false;

    float yaw_deg = 0.0;
    uint16_t yaw_cd = 36000;

    while (inputVideo.grab()) {
        Mat image, imageCopy;
        bool ret = inputVideo.retrieve(image);

        vector<int> ids;
        vector<vector<Point2f>> corners, rejected;

        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // refined strategy to detect more markers
        if (refinedStrategy)
            aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix, distCoeffs);

        // estimate board pose
        int markersOfBoardDetected = 0;
        if (ids.size() > 0)
            markersOfBoardDetected =
                aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec, false);

        poseEstimated = (markersOfBoardDetected > 0 && norm(tvec) > 0.00001);

        now_epoch = system_clock::now().time_since_epoch();
        now_micros = duration_cast<microseconds>(now_epoch).count();
        now_nanos = duration_cast<nanoseconds>(now_epoch).count();
        now = (double)now_micros / 1000000.0;
        
        if (prev_iter > 0.0) {
            dt = (now - prev_iter);
        } else {
            dt = 0.0;
        }

        prev_iter = now;

        if (poseEstimated) {
            Affine3d T, T_inv;
            Matx33d rot_mat_inv;

            T = Affine3d(rvec, Vec3d(tvec[0] - camera_offset_x, tvec[1] - camera_offset_y, tvec[2] - camera_offset_z));

            T = T.rotate(Vec3d(M_PI, 0.0, 0.0));
            T = T.rotate(Vec3d(0.0, 0.0, M_PI_2));

            T_inv = T.inv();
            T_inv = T_inv.rotate(Vec3d(0.0, 0.0, M_PI_2));

            rot_mat_inv = T_inv.rotation();
            tvec_inv = T_inv.translation();
            rvec_inv = rotationMatrixToEulerAngles(rot_mat_inv);

            tra_vec[0] = -tvec_inv[0];
            tra_vec[1] = tvec_inv[1];
            tra_vec[2] = -tvec_inv[2];
            rot_vec[0] = -rvec_inv[0];
            rot_vec[1] = rvec_inv[1];
            rot_vec[2] = -rvec_inv[2];


            // T = Affine3d(rvec, tvec);

            // T_inv = T.inv();

            // tvec_inv = T_inv.translation();
            // rvec_inv = T_inv.rvec();
            // T_inv.rotation(Vec3d(rvec_inv[1], rvec_inv[0], -rvec_inv[2]));
            // rot_mat_inv = T_inv.rotation();
            // rvec_inv = rotationMatrixToEulerAngles(rot_mat_inv);

            // tra_vec[0] = tvec_inv[1];
            // tra_vec[1] = tvec_inv[0];
            // tra_vec[2] = -tvec_inv[2];
            // rot_vec[0] = rvec_inv[0];
            // rot_vec[1] = rvec_inv[1];
            // rot_vec[2] = rvec_inv[2];


            yaw_deg = rot_vec[2] * 180.0 / M_PI;

            yaw_deg = (yaw_deg < 0.0) ? yaw_deg + 360.0 : yaw_deg;
            yaw_deg = (yaw_deg >= 360.0) ? yaw_deg - 360.0 : yaw_deg;

            yaw_cd = (int)(yaw_deg * 100);
            if (yaw_cd == 0)
                yaw_cd = 36000;

            if (dt > 0.0) {
                tvec_vel = (tra_vec - tvec_inv_prev) / dt;
            } else {
                tvec_vel = Vec3d(0.0, 0.0, 0.0);
            }
            
            if (first) {
                tvec_vel_prev = tvec_vel;

                first = false;
            }

            filt_vel = tvec_vel_prev * 0.75 + tvec_vel * 0.25;

            tvec_inv_prev = tra_vec;
            rvec_inv_prev = rot_vec;

            tvec_vel_prev = filt_vel;
        }

        if ((now - prev_send_pos) > pose_msg_period) {
            //cout << "Tick Send Pose = " << (now - last_send_pos) << endl;
            prev_send_pos = now;

            //cout << "vel:" << filt_vel << " yaw: " << yaw_cd << endl;

            if (poseEstimated) {
                //cout << "Send New Vision Position Estimate" <<  endl;
#ifdef MAV
                if (visiongps == 1) {        // position msg
                    send_vision_position_estimate(client.get(), now_micros, tra_vec, rot_vec, reset_counter);
                }
                else if (visiongps == 2) {   // speed msg
                    send_vision_speed_estimate(client.get(), now_micros, filt_vel, reset_counter);
                }
                else if (visiongps == 3) {   // position + speed msg
                    send_vision_position_estimate(client.get(), now_micros, tra_vec, rot_vec, reset_counter);
                    send_vision_speed_estimate(client.get(), now_micros, filt_vel, reset_counter);
                }
                else if (visiongps == 4) {   // gps_in msg
                    send_gps_input(client.get(), now_micros, tra_vec, filt_vel, yaw_cd);
                }

#endif
            } else {
                //cout << "Send Old Vision Position Estimate" <<  endl;
#ifdef MAV
                if (visiongps == 1) {        // position msg
                    send_vision_position_estimate(client.get(), now_micros, tra_vec, rot_vec, reset_counter);
                }
                else if (visiongps == 2) {   // speed msg
                    send_vision_speed_estimate(client.get(), now_micros, {0.0, 0.0, 0.0}, reset_counter);
                }
                else if (visiongps == 3) {   // position + speed msg
                    send_vision_position_estimate(client.get(), now_micros, tra_vec, rot_vec, reset_counter);
                    send_vision_speed_estimate(client.get(), now_micros, {0.0, 0.0, 0.0}, reset_counter);
                }
                else if (visiongps == 4) {   // gps_in msg
                    send_gps_input(client.get(), now_micros, tra_vec, {0.0, 0.0, 0.0}, yaw_cd);
                }

#endif
            }
        }

        if ((now - prev_heartbeat) > 1.0) {
            cout << "Tick Heart Beat = " << (now - prev_heartbeat) << endl;
            prev_heartbeat = now;
#ifdef MAV
            //cout << "Send HB GGO SHP" <<  endl;
			send_heartbeat(client.get());
            send_system_time(client.get(), now_micros);
            send_timesync(client.get(), 0, now_nanos);
			send_gps_global_origin(client.get());
			send_set_home_position(client.get());
#endif
            if (poseEstimated) {
                cout << "tra:" << tra_vec << endl;
                cout << "rot:" << rot_vec << endl;
            }
        }

        // draw results
        if (show_image) {
            image.copyTo(imageCopy);

            if (ids.size() > 0)
                aruco::drawDetectedMarkers(imageCopy, corners, ids);

            if (showRejected && rejected.size() > 0)
                aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

            // if(poseEstimated)
            //     aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);

            imshow("out", imageCopy);

            char key = (char)waitKey(waitTime);
            if (key == 27)
                break;
        }
    }

    return 0;
}
