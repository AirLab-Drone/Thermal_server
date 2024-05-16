

// ! Important!
// If you run this code, you need to run the following line first
// - export LD_LIBRARY_PATH=/home/yuan/server_ws/src/thermal_im4/lib/:$LD_LIBRARY_PATH


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
    int ir_output_w;
    int ir_output_h;
    int ir_output_size;

    // 構造函式
    ThermalData()
    {
        TempMatrix = nullptr;
        Thermal_RGB_Image = nullptr;
        Thermal_Y16_Image = nullptr;
        ir_output_w = 0;
        ir_output_h = 0;
        ir_output_size = 0;
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
const char *server = "192.168.1.168";
const char *username = "admin";
const char *password = "admin123";
const int port = 80;
const int colorbar = 17; 

/* ---------------------------------- 全域變數 ---------------------------------- */
ThermalData thermal_data;





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

    thermal_data.ir_output_w = info.ir_output_w; // 红外通道输出宽   384
    thermal_data.ir_output_h = info.ir_output_h; // 红外通道输出高   288


    if ((thermal_data.ir_output_w != 0) && (thermal_data.ir_output_h != 0))
    {
        thermal_data.ir_output_size = thermal_data.ir_output_w * thermal_data.ir_output_h;
        std::cout << "[Info] Thermal IR Output Width: " << thermal_data.ir_output_w << std::endl; 
        std::cout << "[Info] Thermal IR Output Height: " << thermal_data.ir_output_h << std::endl; 
        std::cout << "[Info] Thermal IR Output Size: " << thermal_data.ir_output_size << std::endl; 
    }

    

    /* ---------------------------------- 分配空間 ---------------------------------- */
    if (thermal_data.TempMatrix == nullptr)
    {
        thermal_data.TempMatrix = (float *)calloc(thermal_data.ir_output_size, sizeof(float));
        std::cout << "[Info] Alloc TempMatrix success" << std::endl; 
    }

    if (thermal_data.Thermal_RGB_Image == nullptr)
    {
        thermal_data.Thermal_RGB_Image = (unsigned char *)malloc(3 * thermal_data.ir_output_size);
        std::cout << "[Info] Alloc Thermal_RGB_Image success" << std::endl; 
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

    const char path[] = "./screenpic.jpg";
    ret = SGP_GetHeatMap(handle, path);
    if (ret == SGP_OK)
    {
        std::cout << "GetHeatMap success" << std::endl;
    }







    // while (true)
    // {
    //     // cv::Mat frameMat = convertRGBToMat(thermal_data.Thermal_RGB_Image);
    //     // cv::imshow("Thermal Image", frameMat);


    //     ret = SGP_GetImageTemps(handle, thermal_data.TempMatrix, thermal_data.ir_output_size, 1);
    //     if (ret != SGP_OK)
    //     {

    //         std::cerr << "[Error] GetImageTemps error" << ret << std::endl;
    //         SGP_Logout(handle);
    //         SGP_UnInitDevice(handle);
    //         return ret;
    //     }

    //     std::cout << "Max Temperature: " << *std::max_element(thermal_data.TempMatrix, thermal_data.TempMatrix + thermal_data.ir_output_size) << std::endl;


        // cv::waitKey(1);
        // if (cv::waitKey(1) == 27)   // ESC to quit
        // {
        //     break;
        // }
    // }

    






   
    // /* ---------------------------------- 定義變數 ---------------------------------- */
    // int IRModelHotSpot_x, IRModelHotSpot_y, IRModelColdSpot_x, IRModelColdSpot_y; // 紅外模組座標
    // int HotSpot_x, HotSpot_y, ColdSpot_x, ColdSpot_y;                             // 輸出圖像座標
    // float HotSpot_temp, ColdSpot_temp;
    // int maxIndex = 0;
    // float maxTemperature = 0.0;
    // float IRModleToRGB_x = static_cast<float>(ir_output_w) / static_cast<float>(ir_model_w);
    // float IRModleToRGB_y = static_cast<float>(ir_output_h) / static_cast<float>(ir_model_h);
    // // std::cout << IRModleToRGB_x << std::endl;
    // // std::cout << IRModleToRGB_y << std::endl;

    // /* ---------------------------------- 分配空間 ---------------------------------- */
    // if (thermal_data.TempMatrix == nullptr)
    // {
    //     thermal_data.TempMatrix = (float *)calloc(ir_output_size, sizeof(float));
    // }

    // if (thermal_data.Thermal_RGB_Image == nullptr)
    // {
    //     thermal_data.Thermal_RGB_Image = (unsigned char *)malloc(3 * ir_output_w * ir_output_h);
    // }

    // if (thermal_data.Thermal_Y16_Image == nullptr)
    // {
    //     thermal_data.Thermal_Y16_Image = (short *)malloc(224256 * sizeof(short));
    // }

    // ret = SGP_OpenIrVideo(handle, GetIrRtsp, nullptr);
    // if (ret != SGP_OK)
    // {
    //     std::cerr << "[Error] OpenIrVideo error" << std::endl;
    //     SGP_Logout(handle);
    //     SGP_UnInitDevice(handle);
    //     return ret;
    // }
    // std::cout << "[Info] OpenIrVideo susss" << std::endl;

    // ret = SGP_GetY16(handle, GetY16Data, nullptr);
    // if (ret != SGP_OK)
    // {
    //     std::cerr << "[Error] GetY16 error" << std::endl;
    //     SGP_Logout(handle);
    //     SGP_UnInitDevice(handle);
    //     return ret;
    // }
    // std::cout << "[Info] GetY16 susss" << std::endl;


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


cv::Mat convertRGBToMat(const unsigned char *rgbData)
{
    cv::Mat frameMat(thermal_data.ir_output_h, thermal_data.ir_output_w, CV_8UC3);
    cv::Mat rgbMat(thermal_data.ir_output_h, thermal_data.ir_output_w, CV_8UC3, const_cast<unsigned char *>(rgbData));
    cv::cvtColor(rgbMat, frameMat, cv::COLOR_RGB2BGR);
    return frameMat.clone();
}

