#include <iostream>
#include <iomanip>
#include <fstream>
#include <algorithm>

#include "imageStream.h"

using namespace std;
using json = nlohmann::json;

void PrintHelp() {
	cout << "Usage :" << endl;
    cout << "    > lpe-tof-ar [parameters json filepath]" << endl;
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
        }
        
        if (cv::waitKey(1) == 27) break;
    }

    std::cout << "done!" << std::endl;
    
    return 0;
}