#include <thread>
#include "DASCRobots.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include "frame_transforms.h"
#include <cmath>
#include <stdlib.h>     /* wcstombs, wchar_t(C) */
#include <boost/core/null_deleter.hpp>

using std::placeholders::_1;
using namespace std::chrono_literals;
// typedef std::make_shared<DASCRobot> dasc_bot_make_shared;
typedef std::shared_ptr<DASCRobot> dasc_bot_shared_ptr;


// Global thread variable
// extern std::thread server_exec_thread_global;

// thread error for moving: https://stackoverflow.com/questions/27473809/error-use-of-deleted-function-stdthreadthreadconst-stdthread
// how to write: https://thispointer.com/c11-how-to-use-stdthread-as-a-member-variable-in-class/
// https://stackoverflow.com/questions/31280145/starting-threads-within-python-extensions
// https://jedyang.com/post/multithreading-in-python-pytorch-using-c++-extension/

class thread_executor{
        public:
            std::vector<std::thread> some_threads;
            rclcpp::executors::MultiThreadedExecutor *server_exec;
    };

// Export functions
extern "C" {

    thread_executor* initialize_threads(){
        return new thread_executor();
    }

    DASCRobot* make_robot_object(wchar_t* robot_name, uint8_t id) {
        char* argv = (char *)malloc( 1000 );
        std::wcstombs(argv, robot_name, 1000);
        std::cout << "inside c++ make 01" << std::endl;
        return new DASCRobot(std::string(argv), id);
    }

    bool init(DASCRobot *robot) { 
        return robot->init(); 
    }

    bool arm(DASCRobot *robot) {return robot->arm();}
    bool disarm(DASCRobot *robot) {return robot->disarm();}
    std::array<double, 3> getWorldPosition(DASCRobot *robot) {return robot->getWorldPosition();}
    std::array<double, 3> getWorldVelocity(DASCRobot *robot) {return robot->getWorldVelocity();}
    std::array<double, 3> getWorldAcceleration(DASCRobot *robot) {return robot->getWorldAcceleration();}
    std::array<double, 3> getBodyAcceleration(DASCRobot *robot) {return robot->getBodyAcceleration();}
    std::array<double, 3> getBodyRate(DASCRobot *robot) {return robot->getBodyRate();}
    bool getBodyQuaternion(DASCRobot *robot, std::array<double, 4>& quat, bool blocking) {
        std::cout << robot->getBodyQuaternion(quat, blocking) << std::endl;
        return robot->getBodyQuaternion(quat, blocking);
    }
    bool setCmdMode(DASCRobot *robot, int mode) {return robot->setCmdMode(static_cast<DASC::ControlMode>(mode));}
    bool cmdWorldPosition(DASCRobot *robot, double x, double y, double z, double yaw, double yaw_rate) {return robot->cmdWorldPosition(x, y, z, yaw, yaw_rate);}
    bool cmdWorldVelocity(DASCRobot *robot, double x, double y, double z, double yaw, double yaw_rate) {return robot->cmdWorldVelocity(x, y, z, yaw, yaw_rate);}
    bool cmdLocalVelocity(DASCRobot *robot, double x, double y, double z, double yaw, double yaw_rate) {
        return robot->cmdLocalVelocity(x, y, z, yaw, yaw_rate);
    }
    bool cmdWorldAcceleration(DASCRobot *robot, double x, double y, double z, double yaw, double yaw_rate) {return robot->cmdWorldAcceleration(x, y, z, yaw, yaw_rate);}
    bool cmdAttitude(DASCRobot *robot, double q_w, double q_x, double q_y, double q_z, double thrust) {return robot->cmdAttitude(q_w, q_x, q_y, q_z, thrust);}
    bool cmdRates(DASCRobot *robot, double roll, double pitch, double yaw, double thrust) {return robot->cmdRates(roll, pitch, yaw, thrust);}
    bool cmdOffboardMode(DASCRobot *robot) {return robot->cmdOffboardMode();}
    void emergencyStop(DASCRobot *robot) {return robot->emergencyStop();}
    void setGPSGlobalOrigin(DASCRobot *robot, double lat, double lon, double alt) {return robot->setGPSGlobalOrigin(lat, lon, alt);}

    // Functions necessary for starting ROS via Python
    std::thread server_exec_thread_global;

    int argc = 1;
    char *argv[1];

    void rosInit(wchar_t* node_name){ //(std::string node_name) {
        // std::wstring ws( args.OptionArg() );
        // std::string test( ws.begin(), ws.end() );
        argv[0] = (char *)malloc( 100 );
        wcstombs(argv[0], node_name, 100);

        setvbuf(stdout, NULL, _IONBF, BUFSIZ);
        rclcpp::init(argc, argv);
        std::cout << "Node: " << argv[0] << " initialized" << std::endl;
    }



    bool rosOk() {return rclcpp::ok();}

    rclcpp::executors::MultiThreadedExecutor* startNodes(DASCRobot* robots[], int nRobots) {
    
        // Add nodes on separate threads
        rclcpp::executors::MultiThreadedExecutor *server_exec  = new rclcpp::executors::MultiThreadedExecutor;
        std::cout << "nrobots: " << nRobots << std::endl;
        // std::cout << "robots: " << robots << std::endl;
        for (int i = 0; i < nRobots; i++){
            std::shared_ptr<DASCRobot> robot = std::shared_ptr<DASCRobot>(robots[i], boost::null_deleter() );
            server_exec->add_node(robot); //(robots[i]);
            std::cout << "Inside c++ 5" << std::endl;
        }
        std::cout << "Inside c++ 1" << std::endl;
        auto server_spin_exec = [server_exec]() {  //[&server_exec]() {
            server_exec->spin();
        };
        std::cout << "Inside c++ 2" << std::endl;
        std::thread server_exec_thread(server_spin_exec);
        server_exec_thread.detach();

        return server_exec;

    }

    std::thread* start_global_thread(  ){
        std::thread* server_exec_thread_global = new std::thread;
        // std::thread server_exec_thread(server_spin_exec);
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        return server_exec_thread_global;
    }



    void closeNodes(){
        server_exec_thread_global.join();
        // server_exec_thread.join();
        rclcpp::shutdown();
    }

    // NEW by Hardik
    void join_global_thread(){
        server_exec_thread_global.join();
    }

    void ros_thread_sleep(int time){ // time in ms
        std::this_thread::sleep_for(std::chrono::milliseconds(time));
    }

}



// // Parameterized Constructor
// ThreadWrapper::ThreadWrapper(std::function<void()> func) : threadHandler(func)
// {}

// // Move Constructor
// ThreadWrapper::ThreadWrapper(ThreadWrapper && obj) : threadHandler(std::move(obj.threadHandler))
// {
//     std::cout << "Move Constructor is called" << std::endl;
// }
// //Move Assignment Operator
// ThreadWrapper & ThreadWrapper::operator=(ThreadWrapper && obj)
// {
//     std::cout << "Move Assignment is called" << std::endl;
//     if (threadHandler.joinable())
//         threadHandler.join();
//     threadHandler = std::move(obj.threadHandler);
//     return *this;
// }

// // Destructor : Join the thread object
// ThreadWrapper::~ThreadWrapper()
// {
//     if (threadHandler.joinable())
//         threadHandler.join();
// }