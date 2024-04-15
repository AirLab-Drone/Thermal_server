#pragma once

#include "SgpParam.h"

SGPSDK_STDC_START

/**
* @brief	初始化一个设备对象
* @param
* @return	返回设备对象
* @note
*/
SGP_API SGP_HANDLE SGP_InitDevice();

/**
* @brief    释放设备对象
* @param
* handle    输入参数，传入设备对象
* @return	无。
* @note
*/
SGP_API void SGP_UnInitDevice(SGP_HANDLE handle);

/**
* @brief            用户登录
* @param
* handle            输入参数，传入设备对象
* server			输入参数，设备服务地址
* username          输入参数，登录用户
* password          输入参数，登录密码
* port				输入参数，端口号（默认80端口）
* @return           成功返回SGP_OK，失败返回错误码
* @note             需要登录以后才能访问其他接口
*/
SGP_API int SGP_Login(SGP_HANDLE handle, const char *server, const char *username, const char *password, int port);

/**
* @brief        用户登出
* @param
* handle        输入参数，传入设备对象
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_Logout(SGP_HANDLE handle);

/**
* @brief        获取通用信息
* @param
* handle        输入参数，传入设备对象
* output        输出参数，获取通用信息
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetGeneralInfo(SGP_HANDLE handle, SGP_GENERAL_INFO *output);

/**
* @brief        开启可见光
* @param
* handle        输入参数，传入设备对象
* callback      输入参数，注册图像回调函数（RGB24数据）
* pUser			输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_OpenVlVideo(SGP_HANDLE handle, SGP_RTSPCALLBACK callback, void *pUser);

/**
* @brief        开启红外
* @param
* handle        输入参数，传入设备对象
* callback      输入参数，注册图像回调函数（RGB24数据）
* pUser			输入参数
* @return       成功返回SGP_OK，失败返回错误码
*/
SGP_API int SGP_OpenIrVideo(SGP_HANDLE handle, SGP_RTSPCALLBACK callback,void *pUser);

/**
* @brief        关闭可见光视频
* @param
* handle        输入参数，传入设备对象
* @return       无
* @note			退出登录会自动关闭视频流
*/
SGP_API void SGP_CloseVlVideo(SGP_HANDLE handle);

/**
* @brief        关闭红外视频
* @param
* handle        输入参数，传入设备对象
* @return       无
* @note			退出登录会自动关闭视频流
*/
SGP_API void SGP_CloseIrVideo(SGP_HANDLE handle);

/**
* @brief        设置电子变倍，只对主码流有效
* @param
* handle        输入参数，传入设备对象
* type			输入参数，类型
* magnification 输入参数，1：红外原始，可见光原始 2：红外2倍，可见光4倍 3：红外3倍，可见光16倍
* @return       成功返回SGP_OK，失败返回错误码
*/
SGP_API int SGP_SetElectronicMagnification(SGP_HANDLE handle, SGP_VIDEO_PARAM_ENUM type, int magnification);

/**
* @brief        获取温度矩阵（医疗机芯有效-仅ZU13A支持）
* @param
* handle        输入参数，传入设备对象
* callback      输入参数，注册温度矩阵回调函数（Short数据，温度*100）
* pUser			输入参数
* @return       成功返回SGP_OK，失败返回错误码
*/
SGP_API int SGP_GetTempMatrix(SGP_HANDLE handle, SGP_TEMPCALLBACK callback, void *pUser);

/**
* @brief        获取Y16
* @param
* handle        输入参数，传入设备对象
* callback      输入参数，注册Y16回调函数
* pUser			输入参数
* @return       成功返回SGP_OK，失败返回错误码
*/
SGP_API int SGP_GetY16(SGP_HANDLE handle, SGP_Y16CALLBACK callback, void *pUser);

/**
* @brief        停止获取Y16
* @param
* handle        输入参数，传入设备对象
* @return       无
*/
SGP_API void SGP_StopY16(SGP_HANDLE handle);

/*
* @brief        Y16输出温度矩阵
* @param
* handle        输入参数，传入设备对象
* dst			输出参数，温度矩阵
* src			输入参数，输入需要的Y16数据
* w				输入参数，输入src的宽
* h				输入参数，输入src的高
* @return       成功返回SGP_OK，失败返回错误码
*/

SGP_API int SGP_GetTempMatrixEx(SGP_HANDLE handle, float *dst, short *src, int w, int h);

/**
* @brief        温度矩阵旋转(short型）
* @param
* handle        输入参数，传入设备对象
* dst			输出参数，输出旋转后的温度矩阵
* src			输入参数，输入需要旋转的温度矩阵
* w				输入参数，输入src的宽
* h				输入参数，输入src的高
* rotation		输入参数，0：旋转90，1：旋转180°，2：旋转270°
* @return       成功返回SGP_OK，失败返回错误码
*/
SGP_API int SGP_GetTempMatriRotation(SGP_HANDLE handle, short *dst, short *src, int w, int h, int rotation);

/**
* @brief        温度矩阵旋转(float型)
* @param
* handle        输入参数，传入设备对象
* dst			输出参数，输出旋转后的温度矩阵
* src			输入参数，输入需要旋转的温度矩阵
* w				输入参数，输入src的宽
* h				输入参数，输入src的高
* rotation		输入参数，0：旋转90，1：旋转180°，2：旋转270°
* @return       成功返回SGP_OK，失败返回错误码
*/
SGP_API int SGP_GetTempMatriRotationEx(SGP_HANDLE handle, float *dst, float *src, int w, int h, int rotation);

/**
* @brief        修改密码
* @param
* handle        输入参数，传入设备对象
* username      输入参数，登录用户名
* oldpassword   输入参数，旧密码
* newpassword   输入参数，新密码
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_ChangePassword(SGP_HANDLE handle, const char *username, const char *oldpassword, const char *newpassword);

/**
* @brief        重置密码
* @param
* handle        输入参数，传入设备对象
* username      输入参数，登录用户名
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_ResetPassword(SGP_HANDLE handle, const char *username);

/**
* @brief        获取单点温度
* @param
* handle        输入参数，传入设备对象
* x				输入参数，横坐标，范围在1到图像宽之间。
* y				输入参数，纵坐标，范围在1到图像高之间。
* output		输出参数，点温度。
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetPointTemp(SGP_HANDLE handle, int x, int y, float *output);

/**
* @brief        获取系统版本信息
* @param
* handle        输入参数，传入设备对象
* output        输出参数，系统版本信息
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetVersionInfo(SGP_HANDLE handle, SGP_VERSION_INFO *output);

/**
* @brief        同步系统时间
* @param
* handle        输入参数，传入设备对象
* datetime      输入参数，同步时间"2020-05-21 12:22:33"
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SynchroTime(SGP_HANDLE handle, const char *datetime);

/**
* @brief        系统重启
* @param
* handle        输入参数，传入设备对象
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_RebootSystem(SGP_HANDLE handle);

/**
* @brief        录制（控制设备端录像）
* @param
* handle        输入参数，传入设备对象
* subtype       输入参数,录制选项 1:开始录制 2：停止录制
* record_stream 输入参数,录制流 1:单光可见光; 2:单光红外; 3:双光，同时录制
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_Record(SGP_HANDLE handle, int subtype,int record_stream);

/**
* @brief        开始录制（本地录像）
* @param
* handle        输入参数，传入设备对象
* type			输入参数，录像类型
* input			输入参数,保存文件路径+文件名+.mp4
* callback		输入参数，录像状态回调，例如自动停止录像
* pUser			输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_StartRecord(SGP_HANDLE handle, SGP_VIDEO_TYPE type, const char *input, SGP_RECORDCALLBACK callback, void *pUser);

/**
* @brief        停止录制（本地录像）
* @param
* handle        输入参数，传入设备对象
* type			输入参数，录像类型
* @return       无
* @note
*/
SGP_API void SGP_StopRecord(SGP_HANDLE handle, SGP_VIDEO_TYPE type);

/**
* @brief        清理数据
* @param
* handle        输入参数，传入设备对象
* @return       成功返回SGP_OK，失败返回错误码
* @note         此接口用于清理磁盘缓存数据
*/
SGP_API int SGP_ClearData(SGP_HANDLE handle);

/**
* @brief        获取分析对象实时温度
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetAnalyticObjectsTemp(SGP_HANDLE handle, SGP_ANALYTIC_TEMPS *output);

/**
* @brief        获取热图
* @param
* handle        输入参数，传入设备对象
* input			输入参数，保存文件路径+文件名+.jpg
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetHeatMap(SGP_HANDLE handle, const char *input);

/**
* @brief        获取高压热图
* @param
* handle        输入参数，传入设备对象
* input			输入参数，保存文件路径+文件名+.fir
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetFirHeatMap(SGP_HANDLE handle, const char *input);

/**
* @brief        获取屏幕截图
* @param
* handle        输入参数，传入设备对象
* type          输入参数,
* input         输入参数，保存文件路径+文件名+.jpg
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetScreenCapture(SGP_HANDLE handle, SGP_IMAGE_TYPE type, const char *input);

/**
* @brief        获取热图(非文件)
* @param
* handle        输入参数，传入设备对象
* input			输入参数，外部分配，获取图片二进制数据
* input_length	输入参数，input大小
* output_length	输出参数，图片二进制数据大小
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetHeatMapCache(SGP_HANDLE handle, char *input, int input_length, int *output_length);

/**
* @brief        获取高压热图(非文件)
* @param
* handle        输入参数，传入设备对象
* input			输入参数，外部分配，获取图片二进制数据
* input_length	输入参数，input大小
* output_length	输出参数，图片二进制数据大小
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetFirHeatMapCache(SGP_HANDLE handle, char *input, int input_length, int *output_length);

/**
* @brief        获取屏幕截图(非文件)
* @param
* handle        输入参数，传入设备对象
* type          输入参数,
* input			输入参数，外部分配，获取图片二进制数据
* input_length	输入参数，input大小
* output_length	输出参数，图片二进制数据大小
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetScreenCaptureCache(SGP_HANDLE handle, SGP_IMAGE_TYPE type, char *input, int input_length, int *output_length);

/**
* @brief        快门操作
* @param
* handle        输入参数，设备对象
* type			输入参数，快门类型
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_DoShutter(SGP_HANDLE handle, SGP_SHUTTER_ENUM type);

/**
* @brief        获取温度矩阵
* @param
* handle        输入参数，传入设备对象
* output        输出参数，输出温度矩阵
* length		输入参数，output大小(要确保输出温度矩阵的长度和length的大小一致）
* type          输入参数，返回的温度矩阵大小：0为推流红外分辨率，1为设备红外原始分辨率
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetImageTemps(SGP_HANDLE handle, float *output, int length, int type);

/**
* @brief        设置测温点索引数组（该接口当前仅ZU08D支持）
* @param
* handle        输入参数，传入设备对象
* index	    输入参数，传入测温点索引数组
* length	    输入参数，测温点索引数组长度(要确保测温点索引数组的长度和length的大小一致）
* type          输入参数，返回的温度矩阵大小：0为推流红外分辨率，1为设备红外原始分辨率
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetTempPoints(SGP_HANDLE handle, int *index, int length, int type);

/**
* @brief        获取测温点数组（该接口当前仅ZU08D支持）
* @param
* handle        输入参数，传入设备对象
* output        输出参数，输出测温点数组对应的温度值数组
* length		输入参数，测温点索引数组长度
* type          输入参数，返回的温度矩阵大小：0为推流红外分辨率，1为设备红外原始分辨率
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetTempPoints(SGP_HANDLE handle, float *output, int length, int type);

/**
* @brief        获取指定矩形区域温度矩阵（该接口当前仅ZU08D支持）
* @param
* handle        输入参数，传入设备对象
* output        输出参数，输出测温点数组对应的温度值数组
* length		输入参数，测温点索引数组长度(要确保测温点数组对应的温度值数组的长度和length的大小一致）
* input         输入参数，指定的矩形区域
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetMatrixTempPoints(SGP_HANDLE handle, float *output, const int &length, const SGP_RECT &input);

/**
* @brief        设置全局测温参数
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetThermometryParam(SGP_HANDLE handle, SGP_THERMOMETRY_PARAM input);

/**
* @brief        获取全局测温参数
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetThermometryParam(SGP_HANDLE handle, SGP_THERMOMETRY_PARAM *output);

/**
* @brief        设置全局测温开关
* @param
* handle        输入参数，传入设备对象
* input         输入参数，0关闭 1开启
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetThermometryFlag(SGP_HANDLE handle, int input);

/**
* @brief        设置色带号
* @param
* handle        输入参数，传入设备对象
* input         输入参数，色带号1~26
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetColorBar(SGP_HANDLE handle, int input);

/**
* @brief        设置色带显示
* @param
* handle        输入参数，传入设备对象
* input         输入参数，0关闭 1开启
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetColorBarShow(SGP_HANDLE handle, int input);

/**
* @brief        设置温度显示类型
* @param
* handle        输入参数，传入设备对象
* input         输入参数，温度显示方式：1 最高温 2 最低温 3 平均温 4 最高温 + 最低温 5 最高温 + 平均温 6 平均温 + 最低温 7 最高温 + 最低温 + 平均温 8不显示
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetTempShowMode(SGP_HANDLE handle, int input);

/**
* @brief        切换测温范围
* @param
* handle        输入参数，传入设备对象
* input         输入参数，0~2（部分设备只有1个档位，目前最多有3个档位，从SGP_GetGeneralInfo中获取当前设备支持档位）（IPT640M 0:-20~150 1:100~350 2:100~550）
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetRange(SGP_HANDLE handle, int input);

/**
* @brief        设置字符串叠加
* @param
* handle        输入参数，传入设备对象
* type			输入参数，是否使用字符叠加 1:关闭; 2,4,5:右下; 3:右上
				// IPM630  1:关闭; 5:右下
				// IPT430M 1:关闭; 5:右下
                // IPT640M 1:关闭; 2:左上; 3:右上; 4:左下; 5:右下
* input			输入参数，字符串
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetStringShow(SGP_HANDLE handle, int type, const char *input);

/**
* @brief        设置分析对象温度显示类型
* @param
* handle        输入参数，传入设备对象
* input         输入参数，对象温度显示:1最高温;2最低温;3平均温;4仅名称;5不显示
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetThermometryRuleShowMode(SGP_HANDLE handle, int input);

/**
* @brief        添加分析对象
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_AddThermometryRule(SGP_HANDLE handle, SGP_RULE input);

/**
* @brief        更新分析对象
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_UpdateThermometryRule(SGP_HANDLE handle, SGP_RULE input);

/**
* @brief        删除分析对象
* @param
* handle        输入参数，传入设备对象
* input         输入参数，分析对象id
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_DeleteThermometryRule(SGP_HANDLE handle, int input);

/**
* @brief        删除全部分析对象
* @param
* handle        输入参数，传入设备对象
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_DeleteAllThermometryRule(SGP_HANDLE handle);

/**
* @brief        获取分析对象
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetThermometryRule(SGP_HANDLE handle, SGP_RULE_ARRAY *output);

/**
* @brief        设置红外图像效果参数
* @param
* handle        输入参数，传入设备对象
* type          输入参数，参数类型
* value         输入参数，参数值
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetIrImageEffectParam(SGP_HANDLE handle, SGP_IR_IMAGE_EFFECT_ENUM type, int value);

/**
* @brief        获取红外图像效果参数
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetIrImageEffectParam(SGP_HANDLE handle, SGP_IAMGE_EFFECT_PARAM_IR_CONFIG *output);

/**
* @brief        设置可见光图像效果参数
* @param
* handle        输入参数，传入设备对象
* type          输入参数，参数类型
* value         输入参数，参数值
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetVlImageEffectParam(SGP_HANDLE handle, SGP_VL_IMAGE_EFFECT_ENUM type, int value);

/**
* @brief        获取可见光图像效果参数
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetVlImageEffectParam(SGP_HANDLE handle, SGP_IAMGE_EFFECT_PARAM_VL_CONFIG *output);

/**
* @brief        设置图像融合
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetImageFusion(SGP_HANDLE handle, SGP_IMAGE_FUSION input);

/**
* @brief        获取图像融合
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetImageFusion(SGP_HANDLE handle, SGP_IMAGE_FUSION *output);

/**
* @brief        设置网络信息
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetNetInfo(SGP_HANDLE handle, SGP_NET_INFO input);

/**
* @brief        获取网络信息
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetNetInfo(SGP_HANDLE handle, SGP_NET_INFO *output);

/**
* @brief        设置端口信息
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetPortInfo(SGP_HANDLE handle, SGP_PORT_INFO input);

/**
* @brief        获取端口信息
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetPortInfo(SGP_HANDLE handle, SGP_PORT_INFO *output);

/**
* @brief        设置录制信息
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetRecordInfo(SGP_HANDLE handle, SGP_RECORD_INFO input);

/**
* @brief        获取录制信息
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetRecordInfo(SGP_HANDLE handle, SGP_RECORD_INFO *output);

/**
* @brief        设置屏蔽区域
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetShieldArea(SGP_HANDLE handle, SGP_SHIELD_AREA_INFO input);

/**
* @brief        获取屏蔽区域
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetShieldArea(SGP_HANDLE handle, SGP_SHIELD_AREA_INFO *output);

/**
* @brief        设置全局温度告警
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetColdHotTrace(SGP_HANDLE handle, SGP_COLD_HOT_TRACE_INFO input);

/**
* @brief        获取全局温度告警
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetColdHotTrace(SGP_HANDLE handle, SGP_COLD_HOT_TRACE_INFO *output);

/**
* @brief        设置分析对象告警
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetTempAlarm(SGP_HANDLE handle, SGP_TEMP_ALARM_INFO input);

/**
* @brief        获取分析对象告警
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetTempAlarm(SGP_HANDLE handle, SGP_TEMP_ALARM_INFO *output);

/**
* @brief        设置视频参数
* @param
* handle        输入参数，传入设备对象
* type          输入参数，参数类型
* value         输入参数，参数值
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetVideoParam(SGP_HANDLE handle, SGP_VIDEO_PARAM_ENUM type, SGP_VIDEO_PARAM input);

/**
* @brief        获取视频参数
* @param
* handle        输入参数，传入设备对象
* type			输入参数，视频类别
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetVideoParam(SGP_HANDLE handle, SGP_VIDEO_PARAM_ENUM type, SGP_VIDEO_PARAM *output);

/**
* @brief        设置网络异常
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetNetException(SGP_HANDLE handle, SGP_NET_EXCEPTION_INFO input);

/**
* @brief        获取网络异常
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetNetException(SGP_HANDLE handle, SGP_NET_EXCEPTION_INFO *output);

/**
* @brief        设置非法访问
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetAccessViolation(SGP_HANDLE handle, SGP_ACCESS_VIOLATION_INFO input);

/**
* @brief        获取非法访问
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetAccessViolation(SGP_HANDLE handle, SGP_ACCESS_VIOLATION_INFO *output);

/**
* @brief        设置邮件信息
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetEmilInfo(SGP_HANDLE handle, SGP_EMAIL_INFO input);

/**
* @brief        获取邮件信息
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetEmilInfo(SGP_HANDLE handle, SGP_EMAIL_INFO *output);

/**
* @brief        设置补光灯信息
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetFillLight(SGP_HANDLE handle, SGP_FILL_LIGHT_INFO input);

/**
* @brief        获取补光灯信息
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetFillLight(SGP_HANDLE handle, SGP_FILL_LIGHT_INFO *output);

/**
* @brief        设置融合状态
* @param
* handle        输入参数，传入设备对象
* input         输入参数,mode红外模式: 0:单光红外; 1:双光红外
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetInfraredMode(SGP_HANDLE handle, int input);

/**
* @brief        获取融合状态
* @param
* handle        输入参数，传入设备对象
* output        输出参数,mode红外模式: 0:单光红外; 1:双光红外
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetInfraredMode(SGP_HANDLE handle, int *output);

/**
* @brief        设置蜂鸣器状态
* @param
* handle        输入参数，传入设备对象
* input         输入参数,silent 0:非静音; 1:静音
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetSilentMode(SGP_HANDLE handle, int input);

/**
* @brief        获取蜂鸣器状态
* @param
* handle        输入参数，传入设备对象
* output        输出参数,silent 0:非静音; 1:静音
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetSilentMode(SGP_HANDLE handle, int *output);

/**
* @brief        设置报警输入
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetAlarmInput(SGP_HANDLE handle, SGP_ALARM_INPUT_INFO input);

/**
* @brief        获取报警输入
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetAlarmInput(SGP_HANDLE handle, SGP_ALARM_INPUT_INFO *output);

/**
* @brief        调焦
* @param
* handle        输入参数，传入设备对象
* type			输入参数
* value			位置
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetFocus(SGP_HANDLE handle, SGP_FOCUS_TYPE type, int value);

/**
* @brief        获取电机位置
* @param
* handle        输入参数，传入设备对象
* output		输出参数，电机位置
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetMotorPosition(SGP_HANDLE handle, int *output);

/**
* @brief        获取校温信息
* @param
* handle        输入参数，传入设备对象
* output		输出参数，校温信息
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetMeasureTempInfo(SGP_HANDLE handle, SGP_MEASURE_TEMP_INFO &output);

/**
* @brief        恢复出厂设置
* @param
* handle        输入参数，传入设备对象
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_FactoryReset(SGP_HANDLE handle);

/**
* @brief        设置串口指令
* @param
* handle        输入参数，传入设备对象
* input			输入参数，串口指令
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SendUart(SGP_HANDLE handle, const char *input);

/**
* @brief        获取成像参数
* @param
* handle        输入参数，传入设备对象
* output		输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetIrImageInfo(SGP_HANDLE handle, SGP_IR_IMAGE_INFO *output);

/**
* @brief        设置成像参数
* @param
* handle        输入参数，传入设备对象
* input			输入参数
* value			输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetIrImageInfo(SGP_HANDLE handle, SGP_IR_IMAGE_INFO_ENUM input, int value);

/**
* @brief        注册温度告警回调函数
* @param
* handle        输入参数，传入设备对象
* callback      输入参数
* pUser			输入参数
* @return       无
* @note
*/
SGP_API void SGP_RegisterTempAlarmCallback(SGP_HANDLE handle, SGP_TEMPALARMCALLBACK callback, void *pUser);

/**
* @brief        注册对象温差告警回调函数
* @param
* handle        输入参数，传入设备对象
* callback      输入参数
* pUser			输入参数
* @return       无
* @note
*/
SGP_API void SGP_RegisterObjTempAlarmCallback(SGP_HANDLE handle, SGP_OBJTEMPALARMCALLBACK callback, void *pUser);

/**
* @brief        注册内存已满回调函数
* @param
* handle        输入参数，传入设备对象
* callback      输入参数
* pUser			输入参数
* @return       无
* @note
*/
SGP_API void SGP_RegisterMemoryFullCallback(SGP_HANDLE handle, SGP_MEMORYFULLCALLBACK callback, void *pUser);

/**
* @brief        注册存储故障回调函数
* @param
* handle        输入参数，传入设备对象
* callback      输入参数
* pUser			输入参数
* @return       无
* @note
*/
SGP_API void SGP_RegisterStorageErrorCallback(SGP_HANDLE handle, SGP_STORAGEERRORCALLBACK callback, void *pUser);

/**
* @brief        注册推流异常回调函数
* @param
* handle        输入参数，传入设备对象
* callback      输入参数
* pUser			输入参数
* @return       无
* @note
*/
SGP_API void SGP_RegisterRtspErrorCallback(SGP_HANDLE handle, SGP_RTSPERRORCALLBACK callback, void *pUser);

/**
* @brief        注册非法访问回调函数
* @param
* handle        输入参数，传入设备对象
* callback      输入参数
* pUser			输入参数
* @return       无
* @note
*/
SGP_API void SGP_RegisterAccessViolationCallback(SGP_HANDLE handle, SGP_ACCESSVIOLATIONCALLBACK callback, void *pUser);

/**
* @brief        注册网络异常回调函数
* @param
* handle        输入参数，传入设备对象
* callback      输入参数
* pUser			输入参数
* @return       无
* @note
*/
SGP_API void SGP_RegisterNetworkErrorCallback(SGP_HANDLE handle, SGP_NETWORKERRORCALLBACK callback, void *pUser);

/**
* @brief        注册外部告警回调函数
* @param
* handle        输入参数，传入设备对象
* callback      输入参数
* pUser			输入参数
* @return       无
* @note
*/
SGP_API void SGP_RegisterAlarmInputCallback(SGP_HANDLE handle, SGP_ALARMINPUTCALLBACK callback, void *pUser);

/**
* @brief        注册火灾报警回调函数
* @param
* handle        输入参数，传入设备对象
* callback      输入参数
* pUser			输入参数
* @return       无
* @note
*/
SGP_API void SGP_RegisterFireAlarmCallback(SGP_HANDLE handle, SGP_FIREALARMCALLBACK callback, void *pUser);

/**
* @brief        注册自动调焦回调函数
* @param
* handle        输入参数，传入设备对象
* callback      输入参数
* pUser			输入参数
* @return       无
* @note
*/
SGP_API void SGP_RegisterAutoFocusCallback(SGP_HANDLE handle, SGP_AUTOFOCUSCALLBACK callback, void *pUser);

/**
* @brief        透传RS485数据查询接口
* @param
* handle        输入参数，传入设备对象
* data			输入参数，传入查询指令
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_CommandSend(SGP_HANDLE handle, const char* data);

/**
* @brief        录制带温度信息的文件(调试中，暂时不写入用户手册）
* @param
* handle        输入参数，传入设备对象
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_StartRecordTempInfo(SGP_HANDLE handle);

/**
* @brief        停止录制带温度信息的文件(调试中，暂时不写入用户手册）
* @param
* handle        输入参数，传入设备对象
* path			输出参数，传出录制的文件路径
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_StopRecordTempInfo(SGP_HANDLE handle, char *path);

/**
* @brief		透传接口(调试中，暂时不写入用户手册）
* @param
* handle        输入参数，传入设备对象
* data			输入参数，传入json数据
* output		输出参数，传出json数据
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_TransparentTransmission(SGP_HANDLE handle, const char* data, char* output);

/**
* @brief		获取已登录用户的token(调试中，暂时不写入用户手册)
* @param
* handle        输入参数，传入设备对象
* token			输入参数，指向 用户 token对应的字符串
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API void SGP_GetUserToken(SGP_HANDLE handle, char* token);

/**
* @brief		上传ARMAPP程序包(调试中，暂时不写入用户手册)
* @param
* handle        输入参数，传入设备对象
* path			输入参数，文件路径
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_UploadAppPkg(SGP_HANDLE handle, char *path);


SGPSDK_STDC_END
