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
* server	        输入参数，设备服务地址
* username          输入参数，登录用户
* password          输入参数，登录密码
* port				输入参数，端口号（默认80端口）
* @return           成功返回SGP_OK，失败返回错误码
* @note             需要登录以后才能访问其他接口，username、password传空字符串，port传0即可
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
* @note         仅ir_output_w、ir_output_h有效
*/
SGP_API int SGP_GetGeneralInfo(SGP_HANDLE handle, SGP_GENERAL_INFO *output);

/**
* @brief        开启红外
* @param
* handle        输入参数，传入设备对象
* callback      输入参数，注册图像回调函数（RGB24数据）
* pUser			输入参数
* @return       成功返回SGP_OK，失败返回错误码
*/
SGP_API int SGP_OpenIrVideo(SGP_HANDLE handle, SGP_RTSPCALLBACK callback, void *pUser);

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
* type			输入参数，类型，当前仅SGP_IR可用
* magnification 输入参数，倍率1、2、4
* @return       成功返回SGP_OK，失败返回错误码
*/
SGP_API int SGP_SetElectronicMagnification(SGP_HANDLE handle, SGP_VIDEO_PARAM_ENUM type, int magnification);

/**
* @brief        获取系统版本信息
* @param
* handle        输入参数，传入设备对象
* output        输出参数，系统版本信息
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅serial、fpga_version、sdk_version有效
*/
SGP_API int SGP_GetVersionInfo(SGP_HANDLE handle, SGP_VERSION_INFO *output);

/**
* @brief        获取分析对象实时温度
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅工业机芯有效
*/
SGP_API int SGP_GetAnalyticObjectsTemp(SGP_HANDLE handle, SGP_ANALYTIC_TEMPS *output);

/**
* @brief        获取热图
* @param
* handle        输入参数，传入设备对象
* input			输入参数，保存文件路径+文件名+.jpg
* @return       成功返回SGP_OK，失败返回错误码
* @note         工业机芯拍国网格式、非工业机芯拍高德格式
*/
SGP_API int SGP_GetHeatMap(SGP_HANDLE handle, const char *input);

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
* length		输入参数，output大小
* type          输入参数，返回的温度矩阵大小：0为推流红外分辨率，1为设备红外原始分辨率
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_GetImageTemps(SGP_HANDLE handle, float *output, int length, int type);

/**
* @brief        设置全局测温参数
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅dist、emiss、humi有效
*/
SGP_API int SGP_SetThermometryParam(SGP_HANDLE handle, SGP_THERMOMETRY_PARAM input);

/**
* @brief        获取全局测温参数
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅dist、emiss、humi有效
*/
SGP_API int SGP_GetThermometryParam(SGP_HANDLE handle, SGP_THERMOMETRY_PARAM *output);

/**
* @brief        设置色带号
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note         input范围1-26
*/
SGP_API int SGP_SetColorBar(SGP_HANDLE handle, int input);

/**
* @brief        切换测温范围
* @param
* handle        输入参数，传入设备对象
* input         输入参数，0~2（部分设备只有1个档位，目前最多有3个档位）
* @return       成功返回SGP_OK，失败返回错误码
* @note
*/
SGP_API int SGP_SetRange(SGP_HANDLE handle, int input);

/**
* @brief        设置分析对象温度显示类型
* @param
* handle        输入参数，传入设备对象
* input         输入参数，对象温度显示:1最高温;2最低温;3平均温;4仅名称;5不显示
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅工业机芯有效
*/
SGP_API int SGP_SetThermometryRuleShowMode(SGP_HANDLE handle, int input);

/**
* @brief        添加分析对象
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅工业机芯有效；仅type、points有效，且type中4无效
*/
SGP_API int SGP_AddThermometryRule(SGP_HANDLE handle, SGP_RULE input);

/**
* @brief        更新分析对象
* @param
* handle        输入参数，传入设备对象
* input         输入参数
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅工业机芯有效；仅type、points有效，且type中4无效
*/
SGP_API int SGP_UpdateThermometryRule(SGP_HANDLE handle, SGP_RULE input);

/**
* @brief        删除分析对象
* @param
* handle        输入参数，传入设备对象
* input         输入参数，分析对象id
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅工业机芯有效
*/
SGP_API int SGP_DeleteThermometryRule(SGP_HANDLE handle, int input);

/**
* @brief        删除全部分析对象
* @param
* handle        输入参数，传入设备对象
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅工业机芯有效
*/
SGP_API int SGP_DeleteAllThermometryRule(SGP_HANDLE handle);

/**
* @brief        获取分析对象
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅工业机芯有效；仅type、points有效，且type中4无效
*/
SGP_API int SGP_GetThermometryRule(SGP_HANDLE handle, SGP_RULE_ARRAY *output);

/**
* @brief        设置红外图像效果参数
* @param
* handle        输入参数，传入设备对象
* type          输入参数，参数类型
* value         输入参数，参数值
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅SGP_IR_BRIGHTNESS、SGP_IR_CONTRAST有效
*/
SGP_API int SGP_SetIrImageEffectParam(SGP_HANDLE handle, SGP_IR_IMAGE_EFFECT_ENUM type, int value);

/**
* @brief        获取红外图像效果参数
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅SGP_IR_BRIGHTNESS、SGP_IR_CONTRAST有效
*/
SGP_API int SGP_GetIrImageEffectParam(SGP_HANDLE handle, SGP_IAMGE_EFFECT_PARAM_IR_CONFIG *output);

/**
* @brief        获取网络信息
* @param
* handle        输入参数，传入设备对象
* output        输出参数
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅dns1、gateway、ipaddr、netmask、mac有效
*/
SGP_API int SGP_GetNetInfo(SGP_HANDLE handle, SGP_NET_INFO *output);

/**
* @brief        设置视频参数
* @param
* handle        输入参数，传入设备对象
* type          输入参数，参数类型，仅SGP_IR有效
* value         输入参数，参数值
* @return       成功返回SGP_OK，失败返回错误码
* @note         仅fps有效
*/
SGP_API int SGP_SetVideoParam(SGP_HANDLE handle, SGP_VIDEO_PARAM_ENUM type, SGP_VIDEO_PARAM input);

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

SGPSDK_STDC_END
