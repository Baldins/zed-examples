///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////

/*********************************************************************************
 ** This sample demonstrates how to capture 3D point cloud and detected objects **
 **      with the ZED SDK and display the result in an OpenGL window. 	        **
 *********************************************************************************/

// Standard includes
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <time.h>       /* time_t, struct tm, time, localtime, strftime */


// Flag to disable the GUI to increase detection performances
// On low-end hardware such as Jetson Nano, the GUI significantly slows
// down the detection and increase the memory consumption
#define ENABLE_GUI 1

// ZED includes
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>
// Sample includes
#include "utils.hpp"

#include <ctime>
// #include "date.h"
// Sample includes
#if ENABLE_GUI
#include "GLViewer.hpp"
#include "TrackingViewer.hpp"
#endif

// Using std and sl namespaces
using namespace std;
using namespace sl;
bool is_playback = false;



void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");
void parseArgs(int argc, char **argv, InitParameters& param);

//std::string datetime()
//{
//    time_t rawtime;
//    struct tm * timeinfo;
//    char buffer[80];
//
//    time (&rawtime);
//    timeinfo = localtime(&rawtime);
//
//    strftime(buffer,80,"%d-%m-%Y %H-%M-%S",timeinfo);
//    return std::string(buffer);
//}



sl::Camera zed;
std::mutex logMutex;


template <typename filename, typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7, typename T8, typename T9, typename T10, typename T11,typename T12>
bool writeCsvFile(filename &fileName, T1 column1, T2 column2, T3 column3,  T4 column4, T5 column5, T6 column6, T7 column7, T8 column8, T9 column9, T10 column10, T11 column11, T12 column12) {
    std::lock_guard<std::mutex> csvLock(logMutex);
    std::fstream file;
    file.open (fileName, std::ios::out | std::ios::app);
    if (file) {
        file << "\""  << column1 << "\",";
        file << "\""  << column2 << "\",";
        file << "\""  << column3 << "\",";
        file << "\""  << column4 << "\",";
        file << "\""  << column5 << "\",";
        file << "\""  << column6 << "\",";
        file << "\""  << column7 << "\",";
        file << "\""  << column8 << "\",";
        file << "\""  << column9 << "\",";
        file << "\""  << column10 << "\",";
        file << "\""  << column11 << "\",";
        file << "\""  << column12 << "\"";
        file <<  std::endl;
        return true;
    } else {
        return false;
    }
}

int main(int argc, char **argv) {
#ifdef _SL_JETSON_
    const bool isJetson = true;
#else
    const bool isJetson = false;
#endif

    if (argc < 2) {
        cout << "Usage : Only the path of the output SVO file should be passed as argument.\n";
        return EXIT_FAILURE;
    }

//    // Enable recording with the filename specified in argument
//    String path (argv[1]);


    // Create ZED objects
    Camera zed;
    InitParameters init_parameters;
    init_parameters.camera_resolution = RESOLUTION::HD1080;
    init_parameters.depth_mode = DEPTH_MODE::NONE;

	init_parameters.sdk_verbose = true;
    init_parameters.camera_fps = 30;


    // On Jetson (Nano, TX1/2) the object detection combined with an heavy depth mode could reduce the frame rate too much
    init_parameters.depth_mode = isJetson ? DEPTH_MODE::PERFORMANCE : DEPTH_MODE::ULTRA;
    init_parameters.depth_maximum_distance = 10.0f * 1000.0f;
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system
    // init_parameters.depth_mode = sl::DEPTH_MODE::NONE; // no depth computation required here
    init_parameters.coordinate_units = UNIT::METER;
	// init_parameters.depth_minimum_distance = 0.15 ; //
	// zed.setDepthMaxRangeValue(40); // Set the maximum depth perception distance to 40m Set
    // init_parameters.depth_maximum_distance = 10.0f * 1000.0f;
    // init_parameters.sensors_required = true;

    parseArgs(argc, argv, init_parameters);


    // Save in the folder

//    std::string path = "../Data/";
    std::string path = argv[1];


    std::string csvFile = path + "output_data.csv";


    writeCsvFile(csvFile, "timestamp", "framerate", "ID", "Label", "Tracking State","Action State", "Position", "Velocity", "Dimensions" , "Detection Confidence", "Camera_Translation", "Camera_Orientation");
    // writeCamera_pose(csvFile_cam, "Translation", "Orientation");


    // Open the camera
    auto returned_state  = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("Camera Open", returned_state, "Exit program.");
        return EXIT_FAILURE;
    }


    // Define the Objects detection module parameters
    ObjectDetectionParameters detection_parameters;
    detection_parameters.enable_tracking = true;
    detection_parameters.enable_mask_output = false; // designed to give person pixel mask
    detection_parameters.detection_model = isJetson ? DETECTION_MODEL::MULTI_CLASS_BOX : DETECTION_MODEL::MULTI_CLASS_BOX_ACCURATE;

    auto camera_config = zed.getCameraInformation().camera_configuration;

    if (detection_parameters.enable_tracking) {
    // Set positional tracking parameters
    PositionalTrackingParameters positional_tracking_parameters;
    positional_tracking_parameters.set_floor_as_origin = true;
 
    // Enable positional tracking
    zed.enablePositionalTracking(positional_tracking_parameters);
	}


    

    // print("Object Detection: Loading Module...");
    returned_state = zed.enableObjectDetection(detection_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        print("enableObjectDetection", returned_state, "\nExit program.");
        zed.close();
        return EXIT_FAILURE;
    }


    // Detection runtime parameters
    // default detection threshold, apply to all object class
    int detection_confidence = 40;
    ObjectDetectionRuntimeParameters detection_parameters_rt(detection_confidence);
    // To select a set of specific object classes:
    detection_parameters_rt.object_class_filter = {OBJECT_CLASS::VEHICLE, OBJECT_CLASS::PERSON, OBJECT_CLASS::ANIMAL};
    // To set a specific threshold
    detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::PERSON] = detection_confidence;
    detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::VEHICLE] = detection_confidence;
    detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::ANIMAL] = detection_confidence;

    // Detection output
    Objects objects;
    Pose zed_pose;
    bool quit = false;



    // Check if the camera is a ZED M and therefore if an IMU is available
    bool zed_has_imu = zed.getCameraInformation().sensors_configuration.isSensorAvailable(sl::SENSOR_TYPE::GYROSCOPE);
    SensorsData sensor_data;



#if ENABLE_GUI

    Resolution display_resolution(min((int)camera_config.resolution.width, 1280) , min((int)camera_config.resolution.height, 720));   
    Resolution tracks_resolution(400, display_resolution.height);
    // create a global image to store both image and tracks view
    cv::Mat global_image(display_resolution.height, display_resolution.width + tracks_resolution.width, CV_8UC4);
    // retrieve ref on image part
    auto image_left_ocv = global_image(cv::Rect(0, 0, display_resolution.width, display_resolution.height));
    // retrieve ref on tracks view part
    auto image_track_ocv = global_image(cv::Rect(display_resolution.width, 0, tracks_resolution.width, tracks_resolution.height));
    // init an sl::Mat from the ocv image ref (which is in fact the memory of global_image)
    Mat image_left(display_resolution, MAT_TYPE::U8_C4, image_left_ocv.data, image_left_ocv.step);
    sl::float2 img_scale(display_resolution.width / (float)camera_config.resolution.width, display_resolution.height / (float)camera_config.resolution.height);

    // 2D tracks  // used init_parameters.camera_fps instead of camera_config.resolution.fps
    TrackingViewer track_view_generator(tracks_resolution, init_parameters.camera_fps, init_parameters.depth_maximum_distance);
    track_view_generator.setCameraCalibration(camera_config.calibration_parameters);    

    string window_name = "ZED| 2D View and Birds view";
    cv::namedWindow(window_name, cv::WINDOW_NORMAL); // Create Window
    cv::createTrackbar("Confidence", window_name, &detection_confidence, 100);

    char key = ' ';
    Resolution pc_resolution(min((int)camera_config.resolution.width, 720) , min((int)camera_config.resolution.height, 404));
    auto camera_parameters = zed.getCameraInformation(pc_resolution).camera_configuration.calibration_parameters.left_cam;
    // Mat point_cloud(pc_resolution, MAT_TYPE::F32_C4, MEM::GPU);
    // GLViewer viewer;
    // viewer.init(argc, argv, camera_parameters);
#endif

    RuntimeParameters runtime_parameters;
    runtime_parameters.measure3D_reference_frame = REFERENCE_FRAME::WORLD;
    runtime_parameters.confidence_threshold = 35;
        
    Pose cam_pose;
    cam_pose.pose_data.setIdentity();
    bool gl_viewer_available=true;


    while (
#if ENABLE_GUI
            gl_viewer_available &&
#endif
            !quit && zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS) {

        // update confidence threshold based on TrackBar
        if(detection_parameters_rt.object_class_filter.empty())
            detection_parameters_rt.detection_confidence_threshold = detection_confidence;
        else // if using class filter, set confidence for each class
            for (auto& it : detection_parameters_rt.object_class_filter)
                detection_parameters_rt.object_class_detection_confidence_threshold[it] = detection_confidence;

        returned_state = zed.retrieveObjects(objects, detection_parameters_rt);

        if ((returned_state == ERROR_CODE::SUCCESS) && objects.is_new) {
#if ENABLE_GUI

            zed.retrieveImage(image_left, VIEW::LEFT, MEM::CPU, display_resolution);

            // as image_left_ocv is a ref of image_left, it contains directly the new grabbed image
            render_2D(image_left_ocv, img_scale, objects.object_list, true);
            zed.getPosition(cam_pose, REFERENCE_FRAME::WORLD);
            // update birds view of tracks based on camera position and detected objects
            track_view_generator.generate_view(objects, cam_pose, image_track_ocv, objects.is_tracked);
// #else


            if (objects.is_tracked) {


                cout << objects.object_list.size() << " Object(s) detected\n\n";

                zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD); 
                            // Get the left image

	                        // Display IMU data
	            if (zed_has_imu) {
	                 // Get IMU data at the time the image was captured
	                zed.getSensorsData(sensor_data, TIME_REFERENCE::IMAGE);

	                //get filtered orientation quaternion
	                auto imu_orientation = sensor_data.imu.pose.getOrientation();
	                // get raw acceleration
	                auto acceleration = sensor_data.imu.linear_acceleration;

	                cout << "IMU Orientation: {" << imu_orientation << "}, Acceleration: {" << acceleration << "}\n";
	            }

                if (!objects.object_list.empty()) {

                		            //                 get the translation information
		            auto zed_translation = zed_pose.getTranslation();
		            // get the orientation information
		            auto zed_orientation = zed_pose.getOrientation();

		            // Display the translation and timestamp
		            cout << "Camera Translation: {" << zed_translation << "}, Orientation: {" << zed_orientation << "}";


                    auto first_object = objects.object_list.front();
                    
                    // auto timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE); // Get image timestamp
                    auto timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE); // Get image timestamp

                    auto framerate = zed.getCurrentFPS();

                    cout << "First object attributes :\n";
                    cout << " Label '" << first_object.label << "' (conf. "
                        << first_object.confidence << "/100)\n";

                    if (detection_parameters.enable_tracking)
                        cout << " Tracking ID: " << first_object.id << " tracking state: " <<
                        first_object.tracking_state << " / " << first_object.action_state << "\n";

                    cout << " 3D position: " << first_object.position <<
                        " Velocity: " << first_object.velocity << "\n";

                    cout << " 3D dimensions: " << first_object.dimensions << "\n";

                    if (first_object.mask.isInit())
                        cout << " 2D mask available\n";

                    cout << " Bounding Box 2D \n";
                    std::ofstream myfile;
                    myfile.open (path  + "bounding_boxes_2D.csv", std::ios::out | std::ios::app);
                    myfile << timestamp << ",";
                    for (auto it : first_object.bounding_box_2d)
                        // cout << "    " << it <<"\n";
                        myfile << it << ",";
                    myfile << "\n";
                    myfile.close();
                    myfile.close();
                    for (auto it : first_object.bounding_box_2d)
                        cout << "    " << it <<"\n";


                    std::ofstream myfile2;
                    myfile2.open (path + "bounding_boxes_3D.csv", std::ios::out | std::ios::app);
                    cout << " Bounding Box 3D \n";
                    myfile2 << timestamp << "," ;
                    for (auto it : first_object.bounding_box)
                        myfile2 << it << "," ;
                    myfile2 << "\n";
                    myfile2.close();
                    for (auto it : first_object.bounding_box)
                        cout << "    " << it <<"\n";

                    // cout << "\nPress 'Enter' to continue...\n";
                    // cin.ignore();
                    cout << "timestamp " << timestamp << "\n";
                    cout << "framerate " << framerate << "\n";


                    if (!writeCsvFile(csvFile, timestamp, framerate, first_object.id, first_object.label , first_object.tracking_state, first_object.action_state, first_object.position,  first_object.velocity, first_object.dimensions, first_object.confidence , zed_translation, zed_orientation)) {
                        std::cerr << "Failed to write to file: " << csvFile << "\n";
                    } 

                    // if (!writeCamera_pose(csvFile, zed_translation, zed_orientation )) {
                    //     std::cerr << "Failed to write to file: " << csvFile_cam << "\n";
                    // } 

                }
            }    

#else
            cout << "Detected " << objects.object_list.size() << " Object(s)" << endl;


#endif
        }

        if (is_playback && zed.getSVOPosition() == zed.getSVONumberOfFrames()) {
            quit = true;
        }

#if ENABLE_GUI
        // gl_viewer_available = viewer.isAvailable();
        // as image_left_ocv and image_track_ocv are both ref of global_image, no need to update it
        cv::imshow(window_name, global_image);
        key = cv::waitKey(10);
        if (key == 'i') {
            track_view_generator.zoomIn();
        } else if (key == 'o') {
            track_view_generator.zoomOut();
        } else if (key == 'q') {
            quit = true;
        } else if (key == 'p') {
            detection_parameters_rt.object_class_filter.clear();
            detection_parameters_rt.object_class_filter.push_back(OBJECT_CLASS::PERSON);
            detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::PERSON] = detection_confidence;
            cout << "Person only" << endl;
        } else if (key == 'v') {
            detection_parameters_rt.object_class_filter.clear();
            detection_parameters_rt.object_class_filter.push_back(OBJECT_CLASS::VEHICLE);
            detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::VEHICLE] = detection_confidence;
            cout << "Vehicle only" << endl;
        } else if (key == 'a') {
            detection_parameters_rt.object_class_filter.clear();
            detection_parameters_rt.object_class_filter.push_back(OBJECT_CLASS::ANIMAL);
            detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::ANIMAL] = detection_confidence;
            cout << "Animal only" << endl;            // detection_parameters_rt.object_class_filter.push_back(OBJECT_CLASS::ANIMAL);

        } else if (key == 'b') {
            detection_parameters_rt.object_class_filter.clear();
            detection_parameters_rt.object_class_filter.push_back(OBJECT_CLASS::PERSON);
            detection_parameters_rt.object_class_filter.push_back(OBJECT_CLASS::ANIMAL);
            detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::PERSON] = detection_confidence;
            detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::ANIMAL] = detection_confidence;
            cout << "People and Animal only" << endl;
        } else if (key == 'c') {
            detection_parameters_rt.object_class_filter.clear();
            detection_parameters_rt.object_class_detection_confidence_threshold.clear();
            cout << "Clear Filters" << endl;
        }
        
#endif
    }
#if ENABLE_GUI
    // viewer.exit();
    // point_cloud.free();
    image_left.free();
#endif
    zed.disableObjectDetection();
    zed.disablePositionalTracking();
    zed.disableRecording();
    zed.close();
    return EXIT_SUCCESS;
}

void print(string msg_prefix, ERROR_CODE err_code, string msg_suffix) {
    cout << "[Sample] ";
    if (err_code != ERROR_CODE::SUCCESS)
        cout << "[Error] ";
    cout << msg_prefix << " ";
    if (err_code != ERROR_CODE::SUCCESS) {
        cout << " | " << toString(err_code) << " : ";
        cout << toVerbose(err_code);
    }
    if (!msg_suffix.empty())
        cout << " " << msg_suffix;
    cout << endl;
}

void parseArgs(int argc, char **argv, InitParameters& param) {
    if (argc > 1 && string(argv[1]).find(".svo") != string::npos) {
        // SVO input mode
        param.input.setFromSVOFile(argv[1]);
        is_playback = true;
        cout << "[Sample] Using SVO File input: " << argv[1] << endl;
    } else if (argc > 1 && string(argv[1]).find(".svo") == string::npos) {
        string arg = string(argv[1]);
        unsigned int a, b, c, d, port;
        if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
            // Stream input mode - IP + port
            string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
            param.input.setFromStream(sl::String(ip_adress.c_str()), port);
            cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << endl;
        } else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
            // Stream input mode - IP only
            param.input.setFromStream(sl::String(argv[1]));
            cout << "[Sample] Using Stream input, IP : " << argv[1] << endl;
        } else if (arg.find("HD2K") != string::npos) {
            param.camera_resolution = RESOLUTION::HD2K;
            cout << "[Sample] Using Camera in resolution HD2K" << endl;
        } else if (arg.find("HD1080") != string::npos) {
            param.camera_resolution = RESOLUTION::HD1080;
            cout << "[Sample] Using Camera in resolution HD1080" << endl;
        } else if (arg.find("HD720") != string::npos) {
            param.camera_resolution = RESOLUTION::HD720;
            cout << "[Sample] Using Camera in resolution HD720" << endl;
        } else if (arg.find("VGA") != string::npos) {
            param.camera_resolution = RESOLUTION::VGA;
            cout << "[Sample] Using Camera in resolution VGA" << endl;
        }
    }
}
