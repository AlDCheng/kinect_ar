#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>

#include "imageStream.h"
#include "kinectfusion.h"

using namespace std;
using json = nlohmann::json;

void PrintHelp() {
	cout << "Usage :" << endl;
    cout << "    > kinect_ar [parameters json filepath]" << endl;
}

// From: https://stackoverflow.com/questions/865668/how-to-parse-command-line-arguments-in-c
class InputParser{
    public:
        InputParser (int &argc, char **argv){
            for (int i=1; i < argc; ++i)
                this->tokens.push_back(std::string(argv[i]));
        }
        const std::string& getCmdOption(const std::string &option) const{
            std::vector<std::string>::const_iterator itr;
            itr =  std::find(this->tokens.begin(), this->tokens.end(), option);
            if (itr != this->tokens.end() && ++itr != this->tokens.end()){
                return *itr;
            }
            static const std::string empty_string("");
            return empty_string;
        }
        bool cmdOptionExists(const std::string &option) const{
            return std::find(this->tokens.begin(), this->tokens.end(), option)
                   != this->tokens.end();
        }
    private:
        std::vector <std::string> tokens;
};

int main(int argc, char* argv[])
{
    // Check for valid json path in command line input
    if (argc < 2) {
        PrintHelp();
        return 1;
    }

    // Get a global configuration (comes with default values) and adjust some parameters
    kinectfusion::GlobalConfiguration configuration;
    configuration.voxel_scale = 2.f;
    configuration.init_depth = 700.f;
    configuration.distance_threshold = 10.f;
    configuration.angle_threshold = 20.f;

    // Create a KinectFusion pipeline with the camera intrinsics and the global configuration
    kinectfusion::CameraParameters cam_params;
    cam_params.focal_x = 582;
    cam_params.focal_y = 582;
    cam_params.image_height = 424;
    cam_params.image_width = 512;
    cam_params.principal_x = 255.5;
    cam_params.principal_y = 211.5;

    kinectfusion::Pipeline pipeline { cam_params, configuration };

    // Parse command line input
    InputParser input(argc, argv);

    // read json
    std::ifstream param_path(argv[1]);
    json params_json;
    param_path >> params_json;
    std::cout << setw(4) << params_json << endl;

    ImageStream streamer(params_json["input"]);

    // Dummy code to test initialization
    while(true) {
        if(!streamer.GetNext()) {
            break;
        } else {
            cv::imshow("RGB", streamer.rgb);
            cv::imshow("Depth", streamer.depth);

            bool success = pipeline.process_frame(streamer.depth, streamer.rgb);
            if (!success)
                std::cout << "Frame could not be processed" << std::endl;
                }
        
        if (cv::waitKey(1) == 27) break;
    }

    std::cout << "done!" << std::endl;

    // Retrieve camera poses
    auto poses = pipeline.get_poses();

    // Export surface mesh
    auto mesh = pipeline.extract_mesh();
    kinectfusion::export_ply("data/mesh.ply", mesh);

    // Export pointcloud
    auto pointcloud = pipeline.extract_pointcloud();
    kinectfusion::export_ply("data/pointcloud.ply", pointcloud);
    
    return 0;
}