


// ! export LD_LIBRARY_PATH=/home/yuan/thermal_camera/Thermal_IPT430M/lib:$LD_LIBRARY_PATH
// set LD_LIBRARY_PATH in launch file
// use "ros2 launch thermal_ipt430m thermal_ipt430m.launch.py " 



extern "C"
{
#include "SgpParam.h"
#include "SgpApi.h"
#include <stdio.h>
#include <sys/types.h>
#include <unistd.h>
#include <malloc.h>
#include <string.h>
#include "sys/time.h"
#include "time.h"
#include <pthread.h>
#include <stdbool.h>
#include <fcntl.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <dirent.h>
#include <libudev.h>
}
#include <iostream>
#include <string>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

/* ----------------------------------- ROS ---------------------------------- */
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32_multi_array.hpp" // for pixel
#include "std_msgs/msg/float32.hpp"           // for temperature
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <std_msgs/msg/string.hpp>
#include "cv_bridge/cv_bridge.h"
#include <image_transport/image_transport.hpp>
#include "std_srvs/srv/set_bool.hpp"

#include "thermal_msgs/srv/auto_focus.hpp" 

using SetBool = std_srvs::srv::SetBool;
using std::placeholders::_1;
using std::placeholders::_2;


/* ---------------------------------- 函式宣告 ---------------------------------- */
static void GetIrRtsp(unsigned char *outdata, int w, int h, void *ptr);
static void GetY16Data(short *y16, int length, void *ptr);
cv::Mat convertY16ToGray(const short *y16Data);
cv::Mat convertRGBToMat(const unsigned char *rgbData);

/* ---------------------------------- 熱像儀輸出結構 ---------------------------------- */
struct ThermalData
{
    // 成員變數
    float *TempMatrix;
    unsigned char *Thermal_RGB_Image;
    short *Thermal_Y16_Image;

    // 構造函式
    ThermalData()
    {
        TempMatrix = nullptr;
        Thermal_RGB_Image = nullptr;
        Thermal_Y16_Image = nullptr;
    }

    // 解構函式
    ~ThermalData()
    {
        if (TempMatrix != nullptr)
        {
            delete[] TempMatrix;
            TempMatrix = nullptr;
        }
        if (Thermal_RGB_Image != nullptr)
        {
            delete[] Thermal_RGB_Image;
            Thermal_RGB_Image = nullptr;
        }
        if (Thermal_Y16_Image != nullptr)
        {
            delete[] Thermal_Y16_Image;
            Thermal_Y16_Image = nullptr;
        }
    }
};

/* ---------------------------------- 設定常量 ---------------------------------- */
const int WIDTH = 512;
const int HEIGHT = 384;
// const int SIZE = WIDTH * HEIGHT; // =196608
const char *server = "192.168.1.168";
const char *username = "admin";
const char *password = "admin123";
const int port = 80;
const int colorbar = 17;

/* ---------------------------------- 全域變數 ---------------------------------- */
ThermalData thermal_data;




/* -------------------------------- ros node -------------------------------- */
class ThermalCameraNode : public rclcpp::Node
{
public:
    ThermalCameraNode(SGP_HANDLE handle, int ir_model_w, int ir_model_h) : Node("thermal_camera_node"), handle_(handle), ir_model_w_(ir_model_w), ir_model_h_(ir_model_h)
    {
        // publisher_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image", 10);

        pixel_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("hot_spot_temperature_pos", 10); // temperature position [x, y]
        image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("thermal_image", 10);                   // thermal rgb img
        temperature_pub_ = this->create_publisher<std_msgs::msg::Float32>("hot_spot_temperature", 10);       // hot spot temperature

        auto_focus_srv_ = this->create_service<thermal_msgs::srv::AutoFocus>("auto_focus", std::bind(&ThermalCameraNode::AutoFocus, this, _1, _2));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(1), std::bind(&ThermalCameraNode::publishThermalData, this));

    }

private:
    // callback funtion
    void publishThermalData()
    {

        if (thermal_data.Thermal_Y16_Image != nullptr)
        {

            SGP_GetTempMatrixEx(handle_, thermal_data.TempMatrix, thermal_data.Thermal_Y16_Image, ir_model_w_, ir_model_h_);

            max_temperature_ = thermal_data.TempMatrix[0];
            for (int i = 0; i < ir_model_w_ * ir_model_h_; i++)
            {
                if (thermal_data.TempMatrix[i] > max_temperature_)
                {
                    max_temperature_ = thermal_data.TempMatrix[i];
                    maxIndex = i;
                }
            }
            IRModelHotSpot_x = (maxIndex) % ir_model_w_ + 1;
            IRModelHotSpot_y = (maxIndex) / ir_model_w_ + 1;

            HotSpot_x = IRModelHotSpot_x * IRModleToRGB_x;
            HotSpot_y = IRModelHotSpot_y * IRModleToRGB_y;


            // std::cout << "================================================" << std::endl;
            // std::cout << "紅外模組熱點x座標: " << IRModelHotSpot_x << std::endl;
            // std::cout << "紅外模組熱點y座標: " << IRModelHotSpot_y << std::endl;
            // std::cout << "計算最熱點x座標: " << HotSpot_x << std::endl;
            // std::cout << "計算最熱點y座標: " << HotSpot_y << std::endl;
            // std::cout << "計算最熱點溫度: " << max_temperature_ << std::endl;
            // std::cout << "================================================" << std::endl;

            
            // Publish pixel values
            auto pixel_msg = std::make_unique<std_msgs::msg::Int32MultiArray>();
            pixel_msg->data = {HotSpot_x, HotSpot_y};
            pixel_pub_->publish(std::move(pixel_msg));
            RCLCPP_INFO(this->get_logger(), "Published pixel values: [%d, %d]", HotSpot_x, HotSpot_y);

            // Publish temperature
            auto temperature_msg = std::make_unique<std_msgs::msg::Float32>();
            temperature_msg->data = max_temperature_;
            temperature_pub_->publish(std::move(temperature_msg));
            RCLCPP_INFO(this->get_logger(), "Published max temperature: %f", max_temperature_);


            if (thermal_data.Thermal_RGB_Image != nullptr)
            {
                cv::Mat RGB_img = convertRGBToMat(thermal_data.Thermal_RGB_Image);
                image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", RGB_img).toImageMsg();

                // Publish the image message
                image_pub_->publish(*image_msg.get());
                // RCLCPP_INFO(this->get_logger(), "Thermal image is published");
            }


        }
    }

    // ros2 service call /auto_focus thermal_msgs/srv/AutoFocus "{auto_focus: 'auto focus'}"
    void AutoFocus(
        const std::shared_ptr<thermal_msgs::srv::AutoFocus::Request> request,
        const std::shared_ptr<thermal_msgs::srv::AutoFocus::Response> response)
    {
        
        std::cout << "Requested Data: " << request->auto_focus << std::endl;

        // If request is start, send velocities to move the robot
        if (request->auto_focus == "auto focus")
        {
            RCLCPP_INFO(this->get_logger(), "auto focus now");
            SGP_SetFocus(handle_, SGP_FOCUS_AUTO, 0);
            response->is_focus = true;
        }
    }

    sensor_msgs::msg::Image::SharedPtr image_msg;
    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr pixel_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temperature_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Service<thermal_msgs::srv::AutoFocus>::SharedPtr auto_focus_srv_;

    int IRModelHotSpot_x = 0;
    int IRModelHotSpot_y = 0;
    int HotSpot_x = 0;
    int HotSpot_y = 0;
    int maxIndex = 0;
    float max_temperature_ = 0.0;
    SGP_HANDLE handle_;
    int ir_model_w_;
    int ir_model_h_;
    float IRModleToRGB_x = 512.0 / 384.0;
    float IRModleToRGB_y = 384.0 / 288.0;
};

int main(int argc, char *argv[])
{

    int ret = 0;
    SGP_HANDLE handle = 0;
    handle = SGP_InitDevice();

    SGP_GENERAL_INFO info;
    memset(&info, 0x00, sizeof(info));

    if (!handle)
    {
        std::cerr << "[Error] Init device error" << std::endl;
        SGP_UnInitDevice(handle);
        return handle;
    }
    std::cout << "[Info] Init device success" << std::endl;

    ret = SGP_Login(handle, server, username, password, port);
    if (ret != SGP_OK)
    {
        std::cerr << "[Error] Login device error" << std::endl;
        SGP_UnInitDevice(handle);
        return ret;
    }
    std::cout << "[Info] Login device success" << std::endl;
    ret = SGP_GetGeneralInfo(handle, &info);

    if (SGP_SetColorBarShow(handle, 0))
    {
        std::cerr << "[Error] Set Color Bar Show error" << std::endl;
    }
    if (SGP_SetColorBar(handle, colorbar))
    {
        std::cerr << "[Error] Set Color Bar error" << std::endl;
    }
    if (SGP_SetTempShowMode(handle, 8))
    {
        std::cerr << "[Error] Set Temp Show Mode error" << std::endl;
    }

    /* --------------------------------- 獲取熱像儀資訊 -------------------------------- */
    int ir_model_w = info.ir_model_w;   // 红外模组宽
    int ir_model_h = info.ir_model_h;   // 红外模组高
    int ir_output_w = info.ir_output_w; // 红外通道输出宽
    int ir_output_h = info.ir_output_h; // 红外通道输出高
    int ir_model_size = 0;
    int ir_output_size = 0;
    if (ir_model_w && ir_model_h)
    {
        ir_model_size = ir_model_w * ir_model_h;
    }
    if (ir_output_w && ir_output_h)
    {
        ir_output_size = ir_output_w * ir_output_h;
    }
    std::cout << "[Info] Infra Model Width: " << ir_model_w << std::endl;    // 384
    std::cout << "[Info] Infra Model Height: " << ir_model_h << std::endl;   // 288
    std::cout << "[Info] Infra Model SIZE: " << ir_model_size << std::endl;  //
    std::cout << "[Info] Infra Output Width: " << ir_output_w << std::endl;  // 512
    std::cout << "[Info] Infra Output Height: " << ir_output_h << std::endl; // 384
    std::cout << "[Info] Infra Model SIZE: " << ir_output_size << std::endl; //

    /* ---------------------------------- 定義變數 ---------------------------------- */
    int IRModelHotSpot_x, IRModelHotSpot_y, IRModelColdSpot_x, IRModelColdSpot_y; // 紅外模組座標
    int HotSpot_x, HotSpot_y, ColdSpot_x, ColdSpot_y;                             // 輸出圖像座標
    float HotSpot_temp, ColdSpot_temp;
    int maxIndex = 0;
    float maxTemperature = 0.0;
    float IRModleToRGB_x = static_cast<float>(ir_output_w) / static_cast<float>(ir_model_w);
    float IRModleToRGB_y = static_cast<float>(ir_output_h) / static_cast<float>(ir_model_h);
    // std::cout << IRModleToRGB_x << std::endl;
    // std::cout << IRModleToRGB_y << std::endl;

    /* ---------------------------------- 分配空間 ---------------------------------- */
    if (thermal_data.TempMatrix == nullptr)
    {
        thermal_data.TempMatrix = (float *)calloc(ir_output_size, sizeof(float));
    }

    if (thermal_data.Thermal_RGB_Image == nullptr)
    {
        thermal_data.Thermal_RGB_Image = (unsigned char *)malloc(3 * ir_output_w * ir_output_h);
    }

    if (thermal_data.Thermal_Y16_Image == nullptr)
    {
        thermal_data.Thermal_Y16_Image = (short *)malloc(224256 * sizeof(short));
    }

    ret = SGP_OpenIrVideo(handle, GetIrRtsp, nullptr);
    if (ret != SGP_OK)
    {
        std::cerr << "[Error] OpenIrVideo error" << std::endl;
        SGP_Logout(handle);
        SGP_UnInitDevice(handle);
        return ret;
    }
    std::cout << "[Info] OpenIrVideo susss" << std::endl;

    ret = SGP_GetY16(handle, GetY16Data, nullptr);
    if (ret != SGP_OK)
    {
        std::cerr << "[Error] GetY16 error" << std::endl;
        SGP_Logout(handle);
        SGP_UnInitDevice(handle);
        return ret;
    }
    std::cout << "[Info] GetY16 susss" << std::endl;





    rclcpp::init(argc, argv);
    auto node = std::make_shared<ThermalCameraNode>(handle, ir_model_w, ir_model_h);
    rclcpp::spin(node);
    rclcpp::shutdown();




    // 釋放
    SGP_Logout(handle);
    SGP_UnInitDevice(handle);
    return ret;
}


static void GetIrRtsp(unsigned char *outdata, int w, int h, void *ptr)
{
    if (outdata)
    {
        // printf("RGB");
        // std::cout << "複製圖片" << std::endl;
        memcpy(thermal_data.Thermal_RGB_Image, outdata, w * h * 3);
    }
}

static void GetY16Data(short *y16, int length, void *ptr)
{
    if (y16)
    {
        // std::cout << length << std::endl;    // 224256
        memcpy(thermal_data.Thermal_Y16_Image, y16, length * sizeof(short));
    }
}

cv::Mat convertRGBToMat(const unsigned char *rgbData)
{
    cv::Mat frameMat(HEIGHT, WIDTH, CV_8UC3);
    cv::Mat rgbMat(HEIGHT, WIDTH, CV_8UC3, const_cast<unsigned char *>(rgbData));
    cv::cvtColor(rgbMat, frameMat, cv::COLOR_RGB2BGR);
    return frameMat.clone();
}

cv::Mat convertY16ToGray(const short *y16Data)
{
    cv::Mat frameMat(HEIGHT, WIDTH, CV_16UC1, const_cast<short *>(y16Data));
    cv::Mat gray8Bit;
    frameMat.convertTo(gray8Bit, CV_8U); // Scale to 0-255
    return gray8Bit;
}
