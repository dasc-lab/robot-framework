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

// Export functions
extern "C" {

    size_t to_narrow(const wchar_t * src, char * dest, size_t dest_len){
        size_t i;
        wchar_t code;

        i = 0;
            std::cout << "inside c++ make 003" << std::endl;
        while (src[i] != '\0' && i < (dest_len - 1)){
            code = src[i];
            if (code < 128){
            dest[i] = char(code);
            std::cout << "inside c++ make 004" << std::endl;
            }
            else{
            dest[i] = '?';
            if (code >= 0xD800 && code <= 0xD8FF)
                // lead surrogate, skip the next code unit, which is the trail
                i++;
                std::cout << "inside c++ make 005" << std::endl;
            }
            i++;
        }
            std::cout << "inside c++ make 006" << std::endl;
        dest[i] = '\0';

        return i - 1;

        }

    // std::shared_ptr<DASC> make_robot( std::string robot_name, uint8_t id ){
    //     return std::make_shared<DASCRobot>(robot_name, id);
    // }

    // DASCRobot* init(std::string robot_name, uint8_t id) {
    DASCRobot* make_robot_object(wchar_t* robot_name, uint8_t id) {
    // dasc_bot_shared_ptr* make_robot_object(wchar_t* robot_name, uint8_t id) {
        std::cout << "inside c++ make 0" << std::endl;
        char* argv = (char *)malloc( 1000 );
        std::cout << "inside c++ make 001" << std::endl;
        // std::cout << "inside c++ make 1" << std::endl;
        // std::cout << "inside c++ make 12" << robot_name << std::endl;
        std::wcstombs(argv, robot_name, 1000);
        // size_t newsize = wcslen(robot_name);
        // std::cout << "inside c++ make 002" << std::endl;
        // // char* argv = (char*)robot_name; 
        // to_narrow( robot_name, argv,  newsize);
        // std::wstring your_wchar_in_ws(robot_name);
        std::cout << "inside c++ make 01" << std::endl;
        // std::string your_wchar_in_str(your_wchar_in_ws.begin(), your_wchar_in_ws.end());
        // std::cout << "inside c++ make 02" << std::endl;
        // std::cout << your_wchar_in_str << std::endl;
        // char* your_wchar_in_char =  your_wchar_in_str.c_str();
        // std::cout << "inside c++ make 2 " << std::string(argv) << std::endl;
        // // std::cout << "Initializing robot: " << argv << std::endl;
        // auto robot = std::make_shared<DASCRobot>( std::string(argv), id );
        // std::cout << "robot: " << robot << std::endl;
        // std::cout << &robot << std::endl;
        // return (&robot);
        return new DASCRobot(std::string(argv), id);
        // return new dasc_bot(std::string(argv), id);
        // return make_robot( std::string(argv), id );
    }

    

    bool init(DASCRobot *robot) { 
        std::cout << "Init: " << robot << std::endl;
        std::cout << "Init: & " << &robot << std::endl;
        // std::cout << "Init: *" << *robot << std::endl;
        return robot->init(); 
    }
    // bool init(dasc_bot_shared_ptr *robot) { 
    //     std::cout << "hello init" << std::endl;
    //     std::cout << robot << std::endl;
    //     std::cout << *robot << std::endl;
    //     (*robot)->init();
    //     std::cout << "hello init 2" << std::endl;
    //     return true; 
    // }
    bool arm(DASCRobot *robot) {return robot->arm();}
    bool disarm(DASCRobot *robot) {return robot->disarm();}
    std::array<double, 3> getWorldPosition(DASCRobot *robot) {return robot->getWorldPosition();}
    std::array<double, 3> getWorldVelocity(DASCRobot *robot) {return robot->getWorldVelocity();}
    std::array<double, 3> getWorldAcceleration(DASCRobot *robot) {return robot->getWorldAcceleration();}
    std::array<double, 3> getBodyAcceleration(DASCRobot *robot) {return robot->getBodyAcceleration();}
    std::array<double, 3> getBodyRate(DASCRobot *robot) {return robot->getBodyRate();}
    bool getBodyQuaternion(DASCRobot *robot, std::array<double, 4>& quat, bool blocking) {return robot->getBodyQuaternion(quat, blocking);}
    bool setCmdMode(DASCRobot *robot, int mode) {return robot->setCmdMode(static_cast<DASC::ControlMode>(mode));}
    bool cmdWorldPosition(DASCRobot *robot, double x, double y, double z, double yaw, double yaw_rate) {return robot->cmdWorldPosition(x, y, z, yaw, yaw_rate);}
    bool cmdWorldVelocity(DASCRobot *robot, double x, double y, double z, double yaw, double yaw_rate) {return robot->cmdWorldVelocity(x, y, z, yaw, yaw_rate);}
    bool cmdLocalVelocity(DASCRobot *robot, double x, double y, double z, double yaw, double yaw_rate) {return robot->cmdLocalVelocity(x, y, z, yaw, yaw_rate);}
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
    // void rosInit(char* node_name){ //(std::string node_name) {
    //     argv[0] = node_name;
    //     setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    //     rclcpp::init(argc, argv);
    // } //void rosInit() {rclcpp::init(1, "")}; // Arguments to rclcpp::init might be different if something in launch file

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

    // array of shared pointers
    // void startNodes(std::shared_ptr<DASCRobot> *robots, int nRobots) {
    // void startNodes(std::vector<DASCRobot*> robots, int nRobots) {
    void startNodes(DASCRobot* robots[], int nRobots) {
    
        // Add nodes on separate threads
        std::cout << "Inside c++ 0" << std::endl;
        rclcpp::executors::MultiThreadedExecutor server_exec;
        std::cout << "nrobots: " << nRobots << std::endl;
        // std::cout << "robots: " << robots << std::endl;
        for (int i = 0; i < nRobots; i++){
            std::cout << "Inside c++ 3 robot: " << robots[i] << std::endl;
            std::shared_ptr<DASCRobot> robot = std::shared_ptr<DASCRobot>(robots[i], boost::null_deleter() );
            std::cout << "Inside c++ 4" << std::endl;
            server_exec.add_node(robot); //(robots[i]);
            std::cout << "Inside c++ 5" << std::endl;
        }
        // std::cout << "Inside c++ 1" << std::endl;
        // auto server_spin_exec = [&server_exec]() {
        //     server_exec.spin();
        // };
        // std::cout << "Inside c++ 2" << std::endl;
        // server_exec_thread_global = std::thread(server_spin_exec); //server_exec_thread_global(server_spin_exec);
        // // std::thread server_exec_thread(server_spin_exec);
        // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        // std::cout << "Nodes started!" << std::endl;

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