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

/*****************************************************************************************
 ** This sample demonstrates how to detect objects and retrieve their 3D position **
 **         with the ZED SDK and display the result in an OpenGL window.                **
 *****************************************************************************************/

 // ZED includes
#include <sl/Camera.hpp>

// Sample includes
#include "GLViewer.hpp"

// Using std and sl namespaces
using namespace std;
using namespace sl;

void print(string msg_prefix, ERROR_CODE err_code = ERROR_CODE::SUCCESS, string msg_suffix = "");
void parseArgs(int argc, char **argv, InitParameters& param);



std::mutex logMutex;


bool fileExists(std::string& fileName) {
    return static_cast<bool>(std::ifstream(fileName));
}

template <typename filename, typename T1, typename T2, typename T3, typename T4, typename T5, typename T6, typename T7, typename T8, typename T9, typename T10>
bool writeCsvFile(filename &fileName, T1 column1, T2 column2, T3 column3,  T4 column4, T5 column5, T6 column6, T7 column7, T8 column8, T9 column9, T10 column10) {
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
        file << "\""  << column10 << "\"";
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

	std::string csvFile = "../output_data.csv";

    // if(!fileExists(csvFile))
    writeCsvFile(csvFile, "timestamp", "framerate", "ID", "Label", "Tracking State","Action State", "Position", "Velocity", "Dimensions" , "Detection Confidence");


	// Create ZED objects
	Camera zed;
	InitParameters init_parameters;
	init_parameters.camera_resolution = RESOLUTION::HD2K;
	// On Jetson the object detection combined with an heavy depth mode could reduce the frame rate too much
	init_parameters.depth_mode = isJetson ? DEPTH_MODE::PERFORMANCE : DEPTH_MODE::ULTRA;
	init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
	init_parameters.coordinate_units = UNIT::METER;
	parseArgs(argc, argv, init_parameters);

	// Open the camera
	auto returned_state = zed.open(init_parameters);
	if (returned_state != ERROR_CODE::SUCCESS) {
		print("Open Camera", returned_state, "\nExit program.");
		zed.close();
		return EXIT_FAILURE;
	}

	// Enable Positional tracking (mandatory for object detection)
	PositionalTrackingParameters positional_tracking_parameters;
	//If the camera is static, uncomment the following line to have better performances and boxes sticked to the ground.
	//positional_tracking_parameters.set_as_static = true;
	returned_state = zed.enablePositionalTracking(positional_tracking_parameters);
	if (returned_state != ERROR_CODE::SUCCESS) {
		print("enable Positional Tracking", returned_state, "\nExit program.");
		zed.close();
		return EXIT_FAILURE;
	}

	// Enable the Objects detection module
	ObjectDetectionParameters obj_det_params;
	obj_det_params.enable_tracking = true;
	obj_det_params.detection_model = isJetson ? DETECTION_MODEL::MULTI_CLASS_BOX : DETECTION_MODEL::MULTI_CLASS_BOX_ACCURATE;

	returned_state = zed.enableObjectDetection(obj_det_params);
	if (returned_state != ERROR_CODE::SUCCESS) {
		print("enable Object Detection", returned_state, "\nExit program.");
		zed.close();
		return EXIT_FAILURE;
	}

	auto camera_info = zed.getCameraInformation().camera_configuration;
	// Create OpenGL Viewer
	GLViewer viewer;
	viewer.init(argc, argv, camera_info.calibration_parameters.left_cam);

	// Configure object detection runtime parameters
	ObjectDetectionRuntimeParameters objectTracker_parameters_rt;
	objectTracker_parameters_rt.detection_confidence_threshold = 35;
	// To select a set of specific object classes, like car, bicycle and bus for instance:
	objectTracker_parameters_rt.object_class_filter = {OBJECT_CLASS::PERSON /*, OBJECT_CLASS::VEHICLE, OBJECT_CLASS::ANIMAL*/ };
	// To set a specific threshold
	objectTracker_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::PERSON] = 35;
	// detection_parameters_rt.object_class_detection_confidence_threshold[OBJECT_CLASS::CAR] = 35;

	// Create ZED Objects filled in the main loop
	Objects objects;
	Mat image;

	// Main Loop
	bool need_floor_plane = positional_tracking_parameters.set_as_static;
	while (viewer.isAvailable()) {
		// Grab images
		if (zed.grab() == ERROR_CODE::SUCCESS) {

			// Retrieve left image
			zed.retrieveImage(image, VIEW::LEFT, MEM::GPU);

			// Retrieve Detected Human Bodies
			zed.retrieveObjects(objects, objectTracker_parameters_rt);

			//Update GL View
			viewer.updateView(image, objects);
		}


		            // if (!objects.object_list.empty()) {
            if (objects.is_tracked) {

                cout << objects.object_list.size() << " Object(s) detected\n\n";

                if (!objects.object_list.empty()) {

                    auto first_object = objects.object_list.front();
                    
                    auto timestamp = zed.getTimestamp(sl::TIME_REFERENCE::IMAGE); // Get image timestamp
                    auto framerate = zed.getCurrentFPS();

                    cout << "First object attributes :\n";
                    cout << " Label '" << first_object.label << "' (conf. "
                        << first_object.confidence << "/100)\n";

                    if (obj_det_params.enable_tracking)
                        cout << " Tracking ID: " << first_object.id << " tracking state: " <<
                        first_object.tracking_state << " / " << first_object.action_state << "\n";

                    cout << " 3D position: " << first_object.position <<
                        " Velocity: " << first_object.velocity << "\n";

                    cout << " 3D dimensions: " << first_object.dimensions << "\n";

                    if (first_object.mask.isInit())
                        cout << " 2D mask available\n";

                    cout << " Bounding Box 2D \n";
                    std::ofstream myfile;
                    myfile.open ("../bounding_boxes_2D.csv", std::ios::out | std::ios::app);
                    myfile << timestamp << ",";
                    for (auto it : first_object.bounding_box_2d)
                        // cout << "    " << it <<"\n";
                        myfile << it << ",";
                    myfile << "\n";
                    myfile.close();
                    for (auto it : first_object.bounding_box_2d)
                        cout << "    " << it <<"\n";


                    std::ofstream myfile2;
                    myfile2.open ("../bounding_boxes_3D.csv", std::ios::out | std::ios::app);
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


                    if (!writeCsvFile(csvFile, timestamp, framerate, first_object.id, first_object.label , first_object.tracking_state, first_object.action_state, first_object.position,  first_object.velocity, first_object.dimensions, first_object.confidence )) {
                        std::cerr << "Failed to write to file: " << csvFile << "\n";
                    } 
                }
            }    

	}

	// Release objects
	image.free();
	objects.object_list.clear();

	// Disable modules
	zed.disableObjectDetection();
	zed.disablePositionalTracking();
	zed.close();
	return EXIT_SUCCESS;
}

void parseArgs(int argc, char **argv, InitParameters& param) {
	if (argc > 1 && string(argv[1]).find(".svo") != string::npos) {
		// SVO input mode
		param.input.setFromSVOFile(argv[1]);
		cout << "[Sample] Using SVO File input: " << argv[1] << endl;
	}
	else if (argc > 1 && string(argv[1]).find(".svo") == string::npos) {
		string arg = string(argv[1]);
		unsigned int a, b, c, d, port;
		if (sscanf(arg.c_str(), "%u.%u.%u.%u:%d", &a, &b, &c, &d, &port) == 5) {
			// Stream input mode - IP + port
			string ip_adress = to_string(a) + "." + to_string(b) + "." + to_string(c) + "." + to_string(d);
			param.input.setFromStream(String(ip_adress.c_str()), port);
			cout << "[Sample] Using Stream input, IP : " << ip_adress << ", port : " << port << endl;
		}
		else if (sscanf(arg.c_str(), "%u.%u.%u.%u", &a, &b, &c, &d) == 4) {
			// Stream input mode - IP only
			param.input.setFromStream(String(argv[1]));
			cout << "[Sample] Using Stream input, IP : " << argv[1] << endl;
		}
		else if (arg.find("HD2K") != string::npos) {
			param.camera_resolution = RESOLUTION::HD2K;
			cout << "[Sample] Using Camera in resolution HD2K" << endl;
		}
		else if (arg.find("HD1080") != string::npos) {
			param.camera_resolution = RESOLUTION::HD1080;
			cout << "[Sample] Using Camera in resolution HD1080" << endl;
		}
		else if (arg.find("HD720") != string::npos) {
			param.camera_resolution = RESOLUTION::HD720;
			cout << "[Sample] Using Camera in resolution HD720" << endl;
		}
		else if (arg.find("VGA") != string::npos) {
			param.camera_resolution = RESOLUTION::VGA;
			cout << "[Sample] Using Camera in resolution VGA" << endl;
		}
	}
}

void print(string msg_prefix, ERROR_CODE err_code, string msg_suffix) {
	cout << "[Sample]";
	if (err_code != ERROR_CODE::SUCCESS)
		cout << "[Error]";
	cout << " " << msg_prefix << " ";
	if (err_code != ERROR_CODE::SUCCESS) {
		cout << " | " << toString(err_code) << " : ";
		cout << toVerbose(err_code);
	}
	if (!msg_suffix.empty())
		cout << " " << msg_suffix;
	cout << endl;
}
