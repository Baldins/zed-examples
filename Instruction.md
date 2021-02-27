# Instructions 

* Download the [ZED SDK 3.4](https://www.stereolabs.com/developers/release/) 
* Install CUDA [11.1](https://developer.nvidia.com/cuda-11.1.0-download-archive) or higher
* Download the [Crowd Perception Github repo](https://github.com/Baldins/crowd_perception.git)

### Pedestrian Tracking 

> #### Build

$ cd Social_Tracking/birds\ eye\ viewer/cpp

$ mkdir build & cd build

$ cmake ..

$ make

> #### Build the storing folder 

$ mkdir "../Data_(***)/"

where (***) = current day and time of the streaming (e.g. "../Data_Saturday_02_21_2021_H_3_00_pm" 

> #### Launch the tracking 

 $ ./ZED_Object_detection_birds_eye_viewer "../Data_(***)/"

where (***) = same as before

> ***Please do remember to do these 2 last step with new time any time you start a new recording***

### Recording 

> #### Build

$ cd svo\ recording/recording/cpp

$ mkdir build & cd build

$ cmake ..

$ make

> #### Launch the recording 

$ ./ZED_Object_detection_birds_eye_viewer "../Data_(***).svo"

where (***) = current day and time of the streaming (e.g. "../Data_Saturday_02_21_2021_H_3_00_pm" 

> ***Please do remember to do this last step with new time any time you start a new recording***
