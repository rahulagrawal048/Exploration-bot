#include <common/lcm_config.h>
#include <mbot/mbot_channels.h>
#include <lcmtypes/robot_path_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include <unistd.h>
#include <cmath>

int main(int argc, char** argv)
{std::cout << "here" <<std::endl;
    int numTimes = 1;
    
    if(argc > 1)
    {
        numTimes = std::atoi(argv[1]);
    }
    
    std::cout << "Commanding robot to drive around 1m square " << numTimes << " times.\n";
    
    robot_path_t path;
    path.path.resize(numTimes * 16);
    
    pose_xyt_t nextPose;
    
    nextPose.x = 1.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[0] = nextPose;
    
    nextPose.x = 1.0f;
    nextPose.y = -1.0f;
    nextPose.theta = -M_PI_2;
    path.path[1] = nextPose;
  
    nextPose.x = 0.0f;
    nextPose.y = -1.0f;
    nextPose.theta = -M_PI;
    path.path[2] = nextPose;
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = M_PI_2;
    path.path[3] = nextPose;
    
    
    nextPose.x = 1.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[4] = nextPose;
    
    nextPose.x = 1.0f;
    nextPose.y = -1.0f;
    nextPose.theta = -M_PI_2;
    path.path[5] = nextPose;
  
    nextPose.x = 0.0f;
    nextPose.y = -1.0f;
    nextPose.theta = -M_PI;
    path.path[6] = nextPose;
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = M_PI_2;
    path.path[7] = nextPose;
    
    nextPose.x = 1.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[8] = nextPose;
    
    nextPose.x = 1.0f;
    nextPose.y = -1.0f;
    nextPose.theta = -M_PI_2;
    path.path[9] = nextPose;
  
    nextPose.x = 0.0f;
    nextPose.y = -1.0f;
    nextPose.theta = -M_PI;
    path.path[10] = nextPose;
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = M_PI_2;
    path.path[11] = nextPose;
    
    nextPose.x = 1.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path[12] = nextPose;
    
    nextPose.x = 1.0f;
    nextPose.y = -1.0f;
    nextPose.theta = -M_PI_2;
    path.path[13] = nextPose;
  
    nextPose.x = 0.0f;
    nextPose.y = -1.0f;
    nextPose.theta = -M_PI;
    path.path[14] = nextPose;
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = M_PI_2;
    path.path[15] = nextPose;
    /*
    nextPose.x = 1.22f;
    nextPose.y = 0.61f;
    nextPose.theta = M_PI_2;
    path.path[4] = nextPose;

    nextPose.x = 1.83f;
    nextPose.y = 0.61f;
    nextPose.theta = 0;
    path.path[5] = nextPose;
    
    nextPose.x = 1.83f;
    nextPose.y = 0.0f;
    nextPose.theta = -M_PI_2;
    path.path[6] = nextPose;
    
    nextPose.x = 1.83f;
    nextPose.y = -0.61f;
    nextPose.theta = -M_PI_2;
    path.path[7] = nextPose;
    
    nextPose.x = 2.44f;
    nextPose.y = -0.61f;
    nextPose.theta = 0.0f;
    path.path[8] = nextPose;
    
    nextPose.x = 2.44f;
    nextPose.y = 0.0f;
    nextPose.theta = M_PI_2;
    path.path[9] = nextPose;
    
    nextPose.x = 3.03f;
    nextPose.y = 0.0f;
    nextPose.theta = 0;
    path.path[10] = nextPose;
    
    nextPose.x = 3.05f;
    nextPose.y = 0.0f;
    nextPose.theta = 0;
    path.path[8] = nextPose;
    
    
    nextPose.x = 0.0f;
    nextPose.y = 1.0f;
    nextPose.theta = -M_PI;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 2] = nextPose;
    }
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = -M_PI_2;
    for(int n = 0; n < numTimes; ++n)
    {
        path.path[4*n + 3] = nextPose;
    }
    */
    // Return to original heading after completing all circuits
//    nextPose.theta = 0.0f;
//    path.path.push_back(nextPose);
    
    nextPose.x = 0.0f;
    nextPose.y = 0.0f;
    nextPose.theta = 0.0f;
    path.path.insert(path.path.begin(), nextPose);
    
    path.path_length = path.path.size();
    
    lcm::LCM lcmInstance(MULTICAST_URL);
	std::cout << "publish to: " << CONTROLLER_PATH_CHANNEL << std::endl;
    lcmInstance.publish(CONTROLLER_PATH_CHANNEL, &path);
    sleep(1);

    return 0;
}
