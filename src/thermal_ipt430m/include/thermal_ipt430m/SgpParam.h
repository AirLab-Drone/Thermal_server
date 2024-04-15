#pragma once

#ifdef _WINDOWS
#define SGP_API __declspec(dllexport)
#else
#define SGP_API
#endif

#ifdef __cplusplus
#    define SGPSDK_STDC_START  extern "C" {
#    define SGPSDK_STDC_END    };
#else
#    define SGPSDK_STDC_START
#    define SGPSDK_STDC_END
#endif

typedef unsigned long long SGP_HANDLE;

#define STRING_LENGH 50
#define RANGE_MAX_NUM 3
#define ANALYTIC_MAX_NUM 21
#define SHIELD_AREA_MAX_NUM 2

#define SGP_OK              (0)//正常
#define SGP_ERR             (1)//错误
#define SGP_ERR_10001       (10001)//消息内容为空
#define SGP_ERR_10002       (10002)//消息内容无效
#define SGP_ERR_10003       (10003)//消息字段为空
#define SGP_ERR_10004       (10004)//无效用户名
#define SGP_ERR_10005       (10005)//未鉴权
#define SGP_ERR_10006       (10006)//密码修改失败
#define SGP_ERR_10007       (10007)//用户无权限操作
#define SGP_ERR_10008       (10008)//用户操作失败
#define SGP_ERR_10009       (10009)//密码错误
#define SGP_ERR_10010       (10010)//用户锁定
#define SGP_ERR_10011       (10011)//用户或密码错误
#define SGP_ERR_10012       (10012)//同版本升级
#define SGP_ERR_10013       (10013)//低版本升级
#define SGP_ERR_10014       (10014)//非法IP
#define SGP_ERR_10015       (10015)//IP冲突
#define SGP_ERR_10016       (10016)//非法子网掩码
#define SGP_ERR_10017       (10017)//非法网管
#define SGP_ERR_10018       (10018)//超过最大在线人数
#define SGP_ERR_10019       (10019)//DHCP错误
#define SGP_ERR_10020       (10020)//密码重复
#define SGP_ERR_10021       (10021)//请求温度矩阵时，正在打快门
#define SGP_ERR_10022       (10022)//请求温度矩阵时，正在切测温范围

struct SGP_RANGE
{
	int id;// 测温档位类型 (低温:0,高温:1,其他:2)
    int min;//测温范围最低温
    int max;//测温范围最高温
};

struct SGP_GENERAL_INFO
{
        char datetime[STRING_LENGH];//系统时间，格式为2020-05-21 12:55:12
        char ir_rtsp_url[STRING_LENGH];//红外主码流rtsp地址
        char ir_sub_rtsp_url[STRING_LENGH];//红外辅码流rtsp地址
        int ir_model_w;//红外模组宽
        int ir_model_h;//红外模组高
        int ir_output_w;//红外通道输出宽
        int ir_output_h;//红外通道输出高
        int range_num;//测温范围数量
        SGP_RANGE range[RANGE_MAX_NUM];//测温范围
        char vl_rtsp_url[STRING_LENGH];//可见光主码流rtsp地址(双光产品支持)
        char vl_sub_rtsp_url[STRING_LENGH];//可见光辅码流rtsp地址(双光产品支持)
};

struct SGP_VERSION_INFO
{
        char model[STRING_LENGH];           //设备型号
        char version[STRING_LENGH];         //系统版本
        char serial[STRING_LENGH];          //序列号
        char fpga_version[STRING_LENGH];    //FPGA版本或ASIC版本
        char measure_version[STRING_LENGH]; //测温版本
        char sdk_version[STRING_LENGH];     //sdk版本
};

struct SGP_RECORD_INFO
{
        int record_interval;//延时录制时间1-3600秒
        int record_max_size;//录制文件最大大小，单位M，1-1000
        int record_time;//录制时长，单位分钟，1-60
};

enum SGP_VIDEO_TYPE
{
        SGP_VL_VIDEO = 1,//可见光录像
        SGP_IR_VIDEO = 2,//红外录像
};

enum SGP_IMAGE_TYPE
{
        SGP_VL_IMAGE = 1,//可见光图片
        SGP_IR_IMAGE = 2,//红外图片
};

struct SGP_ANALYTIC_TEMP
{
        int rule_id;    //规则id
        char rule_name[STRING_LENGH];  //规则名称 32字符以内
        int type;//对象类型 1点; 2线; 3矩形; 4多边形; 5圆形
        float max_temp;//最高温度值
        float min_temp;//最低温度值
        float avg_temp;//平均温度值
};

struct SGP_ANALYTIC_TEMPS
{
        int analytic_num;
        SGP_ANALYTIC_TEMP analytic[ANALYTIC_MAX_NUM];
		float global_max_temp;//全局最高温度值
		float global_min_temp;//全局最低温度值
		float global_avg_temp;//全局平均温度值
};

struct SGP_THERMOMETRY_PARAM
{
        int color_bar;//色带1-26
        int color_show;//色带显示0~1
        int flag;//测温开关0~1
        float mod_temp;//温度修正
        int show_mode;//温度显示方式：1 最高温 2 最低温 3 平均温 4 最高温 + 最低温 5 最高温 + 平均温 6 平均温 + 最低温 7 最高温 + 最低温 + 平均温 8不显示
        int gear;//测温范围
        int show_string; //是否使用字符叠加 1:关闭; 2, 4, 5:右下; 3:右上
                         //IPM630 1:关闭; 5:右下
                         //IPM640M 1:关闭; 2:左上; 3:右上; 4:左下; 5:右下
        char show_desc[STRING_LENGH];//显示字符串
        float atmo_trans;//大气透过率0.01-1
        float dist;//距离，单位米，0.1-20.0
        float emiss;//发射率 0.1-1.0
        int emiss_mode;//发射率类型:1标准;2自定义
        int humi;//湿度,范围1-100
        float opti_trans;//光学透过率0.01-1
        float ref_temp;//反射温度-20~550,单位摄氏度(web2.0的反射温度范围: -40-2000)
        int isot_flag;//等温线开关0:关闭;1开启(工业机芯支持)
        float isot_high;//高温阈值0~400
        char isot_high_color[STRING_LENGH];//高温颜色,十六进制值,如红色:0xff0000
        int isot_low;//低温阈值-50~-100
        char isot_low_color[STRING_LENGH];//低温颜色,十六进制值,如红色:0xff0000
        int isot_type;//范围类型:1 关闭等温线效果 2 开启高等温线 3 开启低等温线 4 开启区间内等温线 5 开启区间外等温线
        float ambient;//环境温度
};

struct SGP_POINT
{
        int x;//x坐标 范围 参照红外图像
        int y;//y坐标 范围 参照红外图像
};

enum SGP_SHUTTER_ENUM
{
        SGP_SHUTTER = 1,//快门操作
        SGP_SHUTTER_OPEN = 2,//快门常开
        SGP_SHUTTER_CLOSE = 3,//快门常闭
        SGP_SHUTTER_AUTO = 4,//自动快门
};

struct SGP_RULE
{
	int id;//分析对象id，内部分配，无需设置。
	int alarm_condition;//报警条件:1高于;2低于;3匹配;4高于和低于(web2.0支持1和2)
	int alarm_flag;//是否报警:0不需要;1需要
	int alarm_time;//去抖动时间,0-10秒
	int alarm_interal;// 报警间隔时间，单位：秒。允许设置的间隔时长为：30， 60, 300， 600，900,1800， 3600
	int alarm_type;//报警类型:1高温报警;2低温报警;3平均温报警;4最高温+最低温报警(web2.0 只支持1、2、3)
	float avg_temp;//平均温(基于设备的测温范围)
	int flag;//是否启用配置:0不启用;1启用
	float high_temp;//报警高温阈值(基于设备的测温范围)
	float low_temp;//报警低温阈值(基于设备的测温范围)
	int points_num;
	SGP_POINT points[7];//矩形，圆是四个点，顺时针顺序
	char rule_name[STRING_LENGH];//规则名称，支持50字符
	int show_location;//名称显示位置:1上方;2下方;3左方;4右方;5中间
	float temp_mod;//温度误差
	int type;//对象类型:1点;2线;3矩形;4多边形;5圆
	float atmo_trans;//大气透过率0.01-1
	float dist;//距离，单位米，0.1-20.0
	float emiss;//发射率 0.1-1.0
	int emiss_mode;//发射率类型:1标准;2自定义
	int humi;//湿度,范围1-100
	float opti_trans;//光学透过率0.01-1
	float ref_temp;//反射温度-20~550,单位摄氏度(web2.0 -40到2000)
	int show_type;//显示内容，范围1~8,1~5分别表示：1最高温，2最低温，3平均温度，4仅名称，5不显示。6~8属于预留部分
};

// 联调时发现Web端的请求体的数据结构如下
// 弃用
struct SGP_RULE_NEW
{
	int id;//分析对象id，内部分配，无需设置。
	int alarm_condition;//报警条件:1高于;2低于;3匹配;4高于和低于(web2.0支持1和2)
	int alarm_flag;//是否报警:0不需要;1需要
	int alarm_time;//去抖动时间,0-10秒
	int alarm_type;//报警类型:1高温报警;2低温报警;3平均温报警;4最高温+最低温报警(web2.0 只支持1、2、3)
	int alarm_interal; // 报警间隔时间，单位：秒。允许设置的间隔时长为：30， 60, 300， 600，900,1800， 3600
	float avg_temp;//平均温-20~550
	int flag;//是否启用配置:0不启用;1启用
	float high_temp;//报警高温阈值,-20~550
	float low_temp;//报警低温阈值,-20~550
	int points_num;
	SGP_POINT points[7];//矩形，圆是四个点，顺时针顺序
	char rule_name[STRING_LENGH];//规则名称，支持50字符
	int show_location;//名称显示位置:1上方;2下方;3左方;4右方;5中间
	float temp_mod;//温度误差
	int type;//对象类型:1点;2线;3矩形;4多边形;5圆
	float atmo_trans;//大气透过率0.01-1
	float dist;//距离，单位米，0.1-20.0
	float emiss;//发射率 0.1-1.0
	int emiss_mode;//发射率类型:1标准;2自定义
	int humi;//湿度,范围1-100
	float opti_trans;//光学透过率0.01-1
	float ref_temp;//反射温度-20~550,单位摄氏度(web2.0 -40到2000)
	int show_type;//显示内容，范围1~8,1~5分别表示：1最高温，2最低温，3平均温度，4仅名称，5不显示。6~8属于预留部分
	float diff_temp;
	int preset;
	int rate_temp;
};

//矩阵坐标，4个点，顺时针顺序
struct SGP_MATRIX_RULE
{
	SGP_POINT points[4];//矩形，四个点，顺时针顺序
};

struct SGP_RULE_ARRAY
{
        int rule_num;
        SGP_RULE rule[ANALYTIC_MAX_NUM];//规则列表
};

struct SGP_RULE_ARRAY_NEW
{
	int rule_num;
	SGP_RULE_NEW rule[ANALYTIC_MAX_NUM];//规则列表
};

enum SGP_IR_IMAGE_EFFECT_ENUM
{
        SGP_IR_AUTO_SHUTTER = 1,//快门
        SGP_IR_BRIGHTNESS = 2,//亮度
        SGP_IR_CONTRAST = 3,//对比度
        SGP_IR_REVERSE = 4,//是否反转
        SGP_IR_TIME_FLAG = 5,//降噪时域滤波开关
        SGP_IR_TIME_VALUE = 6,//降噪时域滤波值
        SGP_IR_SPACE_FLAG = 7,//降噪空域滤波开关
        SGP_IR_SPACE_VALUE = 8,//降噪空域滤波值
        SGP_IR_IEE_FLAG = 9,//细节增强开关
        SGP_IR_IEE_VALUE = 10,//细节增强值
        SGP_IR_SATURATION = 11,//饱和度
        SGP_IR_SHARPNESS = 12,//锐度
		SGP_IR_ROTATE = 13,//旋转
};

struct SGP_IAMGE_EFFECT_PARAM_IR_CONFIG
{
        int auto_shutter;//快门自动补偿时间1-20(单位分钟)
        int brightness;//亮度，取值范围0-100
        int contrast;//对比度，取值范围0-100
        int reverse;//是否反转，0:不反转 1 反转
        int time_flag;//降噪时域滤波开关:0关闭;1开启
        int time_value;//降噪时域滤波值 0-100
        int space_flag;//降噪空域滤波开关:0关闭;1开启
        int space_value;//降噪空域滤波值 0-100
        int iee_flag;//细节增强开关:0关闭;1开启
        int iee_value;//细节增强值0-100
        int saturation;//饱和度，取值范围0-100(红外设备不支持)
        int sharpness;//锐度，取值范围0-100
		int mirror_horizontal;//是否开启水平镜像（开启：1，关闭：0）
		int rotate;//旋转参数（顺时针，0:0°，1:90°，2： 180°，3:270°）
};

enum SGP_VL_IMAGE_EFFECT_ENUM
{
        SGP_VL_BLC = 1,//背光补偿:0关闭; 1上; 2下; 3左; 4右; 5中; 6自动
        SGP_VL_BRIGHTNESS = 2,//亮度，取值范围0-100
        SGP_VL_CONTRAST = 3,//对比度，取值范围0-100
        SGP_VL_EXP = 4,//曝光补偿：0-100
        SGP_VL_HLC = 5,//强光抑制:0关闭;1开启
        SGP_VL_REVERSE = 6,//是否反转，0:不反转 1 反转
        SGP_VL_SATURATION = 7,//饱和度，取值范围0-100
        SGP_VL_SHARPNESS = 8,//锐度，取值范围0-100
        SGP_VL_WDR = 9,//宽动态 0:关闭; 1:20%; 2:40%; 3:60%; 4:80%; 5:100%
};

struct SGP_IAMGE_EFFECT_PARAM_VL_CONFIG
{
        int blc;//背光补偿:0关闭; 1上; 2下; 3左; 4右; 5中; 6自动
        int brightness;//亮度，取值范围0-100
        int contrast;//对比度，取值范围0-100
        int exp;//曝光补偿：0-100
        int hlc;//强光抑制:0关闭;1开启
        int reverse;//是否反转，0:不反转 1 反转
        int saturation;//饱和度，取值范围0-100
        int sharpness;//锐度，取值范围0-100
        int wdr;//宽动态 0:关闭; 1:20%; 2:40%; 3:60%; 4:80%; 5:100%
};

struct SGP_IMAGE_FUSION_MATCH_POINTS
{
        SGP_POINT point1;//第1个校准点
        SGP_POINT point2;//第2个校准点
        SGP_POINT point3;//第3个校准点
        SGP_POINT point4;//第4个校准点
        SGP_POINT point5;//第5个校准点
};

struct SGP_IMAGE_FUSION
{
        int percent;//融合比例值0-100
        int ir_left;//红外图像左边裁剪像素值0～50
        int ir_right;//红外图像右边裁剪像素值0～50
        int ir_top;//红外图像上边裁剪像素值0～50
        int ir_botton;//红外图像下边裁剪像素值0～50
        int vl_left;//可见光图像左边裁剪像素值0～1000
        int vl_right;//可见光图像右边裁剪像素值0～1000
        int vl_top;//可见光图像上边裁剪像素值0～1000
        int vl_botton;//可见光图像下边裁剪像素值0～1000
        SGP_IMAGE_FUSION_MATCH_POINTS ir_match_points;//红外校准点
        SGP_IMAGE_FUSION_MATCH_POINTS vl_match_points;//可见光校准点
};

struct SGP_NET_INFO
{
        int card;//网卡类型:0有线网卡
        char dns1[STRING_LENGH];//dns服务器xxx.xxx.xxx.xxx
        char dns2[STRING_LENGH];//dns服务器xxx.xxx.xxx.xxx
        char gateway[STRING_LENGH];//网关xxx.xxx.xxx.xxx
        char host_name[STRING_LENGH];//主机名
        int ip_version;//版本 0:ipv4; 1:ipv6
        char ipaddr[STRING_LENGH];//网络ip地址xxx.xxx.xxx.xxx
        char mac[STRING_LENGH];//Mac地址
        int mode;//模式 0:静态; 1:动态
        char netmask[STRING_LENGH];//子网掩码xxx.xxx.xxx.xxx
};

struct SGP_PORT_INFO
{
        int http_port;//http服务器端口，默认端口80保留设置
        int max_connectios;//最大web连接数， 1-20
        int onvif_check;//Onvif登录校验 0:不校验; 1:校验
        int rtsp_port;//红外rtsp端口，1024-65535，端口用于rtsp流服务，默认端口554保留设置
        int tcp_port;//web消息交互端口，不可设置
};

struct SGP_RECT
{
        int x;//x坐标，1-图像宽
        int y;//y坐标， 1-图像高
        int w;//区域宽，与坐标共同作用，取值范围1-图像宽
        int h;//区域高，与坐标共同作用，取值范围1-图像高
};

struct SGP_SHIELD_AREA_INFO
{
        int rect_num;
        SGP_RECT rect[SHIELD_AREA_MAX_NUM];//区域数组,左上角坐标0, 0标准,最多支持两个
};

struct SGP_PERIOD
{
        char start[STRING_LENGH];//开始时间，格式 HH:mm:ss
        char end[STRING_LENGH];//结束时间，格式 HH:mm:ss
};

struct SGP_EFFECT_DAY
{
        int day;//1-7,星期几
        int period_num;//时间段数量
        SGP_PERIOD period[6];//时间段
};


struct SGP_COLD_HOT_TRACE_INFO
{
        int light_hold;//闪光灯持续时间 10-300s
        int light_flag;//是否开启闪光灯 0:否; 1:是
        int alarm_shake;//报警抖动,单位s,0-100
        int capture_flag;//是否截图 0:否; 1:是
        int capture_stream;//截图类型 1:只截图可见光; 2:只截图红外; 3:截图红外和可见光(web2.0 截图类型 0:不截图)
        char high_color[STRING_LENGH];//高温颜色:0xRGB
        int high_flag;//高温是否检测 0:不检测; 1:检测
        float high_temp;//高温温度，-40~2000
        char low_color[STRING_LENGH];//低温颜色:0xRGB
        int low_flag;//低温是否检测 0:不检测; 1:检测
        float low_temp;//低温温度，-40~2000
        int record_delay;//录像时间 10-300s
        int record_flag;//是否录制 0:不录制; 1:录制
        int record_stream;//录制类型 1:只录制可见光; 2:只录制红外; 3:录制红外和可见光(web2.0 录制类型 0:不录制)
        int sendmail;//是否发送邮件 0:不发送; 1:发送
        int trace_flag;//是否开启 0:不开启; 1:开启
        int output_flag;//是否外部输出 0:不输出 1:输出
        int output_hold;//外部输出持续时间 10-300s
        int audio_flag;//是否音乐提醒 1:是; 0:否
        int audio_index;//音乐文件索引，0-2
        int audio_mode;//音乐播放模式 1:播放次数; 2:持续时间
        int audio_value;//音乐播放值,随模式定义:(持续时间:秒数)(播放次数:播放次数0-100)
        int effect_day_num;//时间数组数量
        SGP_EFFECT_DAY effect_day[7];//时间数组
		int high_condition;//全局最高温对应的控制条件, 1:大于，0:小于
		int low_condition;//全局最低温对应的控制条件, 1:大于，0:小于
};

struct SGP_TEMP_ALARM_INFO
{
        int audio_flag;//是否音乐提醒 1:是; 0:否
        int audio_index;//音乐文件索引，0-2
        int audio_mode;//音乐播放模式 1:播放次数; 2:持续时间
        int audio_value;//音乐播放值,随模式定义:(持续时间:秒数)(播放次数:播放次数0-100)
        int alarm_flag;//是否开启报警 1:开启; 0:不开启(新web上该字段弃用)
        int light_hold;//闪光灯持续时间，10-300s
        int light_flag;//是否开启闪光灯 0:否; 1:是
        int alarm_shake;//报警抖动0-100s(新web上仅在全局温度-报警参数设置-去抖动，功能上使用)
        int capture_flag;//是否截图 0:否; 1:是（web2.0上该字段弃用）
        int capture_stream;//截图类型 0:不截图; 1:只截图可见光; 2:只截图红外; 3:截图红外和可见光
        int record_delay;//录制时间 10-300s
        int record_flag;//是否录制 0:不录制; 1:录制（web2.0上该字段弃用）
        int record_stream;//录制类型 0:不录制; 1:只录制可见光; 2:只录制红外; 3:录制红外和可见光
        int sendmail;//是否发送邮件 0:不发送; 1:发送
        int output_flag;//是否外部输出 0:不输出 1:输出
        int output_hold;//外部输出持续时间10-300s
        int effect_day_num;//时间数组数量
        SGP_EFFECT_DAY effect_day[7];//时间数组
};

struct SGP_ALARM_INPUT_INFO
{
        int flag;//是否开启 0 不开启 1 开启
        int alarm_shake;//报警抖动0-100s
        int type;//输入类型：0 常开型  1 常闭型
        int record_delay;//录制延时 10-300
        int record_flag;//是否录制 0:不录制; 1:录制
        int record_stream;//录制类型 0:不录制; 1:只录制可见光; 2:只录制红外; 3:录制红外和可见光
        int capture_flag;//是否截图 0:否; 1:是
        int capture_stream;//截图类型 0:不截图; 1:只截图可见光; 2:只截图红外; 3:截图红外和可见光
        int sendmail;//是否发送邮件 0:不发送; 1:发送
        int light_flag;//是否开启闪光灯 0:否; 1:是
        int light_hold;//闪光灯持续时间，10-300s
        int output_flag;//是否外部输出 0:不输出 1:输出
        int output_hold;//外部输出持续时间10-300s
        int audio_flag;//是否音乐提醒 1:是; 0:否
        int audio_index;//音乐文件索引，0-2
        int audio_mode;//音乐播放模式 1:播放次数; 2:持续时间
        int audio_value;//音乐播放值,随模式定义:(持续时间:秒数)(播放次数:播放次数0-100)
        int effect_day_num;//时间数组数量
        SGP_EFFECT_DAY effect_day[7];//时间数组
};

enum SGP_VIDEO_PARAM_ENUM
{
        SGP_VL = 1,//可见光主码流
        SGP_VL_SUB = 2,//可见光辅码流
        SGP_IR = 3,//红外主码流
        SGP_IR_SUB = 4,//红外辅码流
};

struct SGP_VIDEO_PARAM
{
        int bit_size;//主码流固定码流值，可变码流时也需设置（宽 * 高 * fps * 8 * 3 / 2 /1024 / 压缩率）其中压缩率范围（18-500） 如分辨率1280x720 取值范围：540Kb/s - 15000Kb/s
        int encodec;//主码流编码 0:h264; 1:h265; 2:mjpeg
        int fps;//主码流帧率1-25
        int gop_size;//主码流帧间隔1-50
        int level;//编码质量等级，等级效果随实际变化，如使用ffmpeg，需服务端自映射（默认medium，可以上下延续几个等级）1:最好 2:更好 3:好 4:差 5:更差 6:最差
        int rate_control;//主码流控制 0:可变码流;1:固定码流
        char ratio[STRING_LENGH];/*主码流分辨率
                                                         1920x1080
                                                         1280x960
                                                         1280x720
                                                         可见光辅码流分辨率
                                                         704x576
                                                         640x480
                                                         红外主码流分辨率
                                                         512x384(640*512)
                                                         红外辅码流分辨率
                                                         (384*288)256x192*/
        int svc;//帧率可分层编码，H264有效，其他格式也需传入 0:分层编码; 1:不分层编码
};

enum SGP_FOCUS_TYPE
{
        SGP_FOCUS_STOP = 0,//电机停止
        SGP_FOCUS_FAR = 1,//远焦
        SGP_FOCUS_NEAR = 2,//近焦
        SGP_FOCUS_FAR_FINE = 3,//远焦微调
        SGP_FOCUS_NEAR_FINE = 4,//近焦微调
        SGP_FOCUS_AUTO = 5,//自动聚焦
        SGP_FOCUS_PLACE = 6,//设置位置
};

struct SGP_NET_EXCEPTION_INFO
{
        int audio_flag;//是否音频联动 0:否; 1:是
        int audio_index;//音频文件索引0-2
        int audio_mode;//音频模式 0:持续时间; 1:播放次数
        int audio_value;//音频模式值 0-100（次/秒）
        int flag;//是否开启 0:不开启; 1:开启
        int light_flag;//是否闪光灯 0:否; 1:是
        int light_hold;//闪光灯持续时间10-300s
        int output_flag;//是否外部输出 0:不输出 1:输出
        int output_hold;//外部输出持续时间1-300s
};

struct SGP_ACCESS_VIOLATION_INFO
{
        int audio_flag;//是否音频联动 0:否; 1:是
        int audio_index;//音频文件索引0-2
        int audio_mode;//音频模式 0:持续时间; 1:播放次数
        int audio_value;//音频模式值 0-100（次/秒）
        int allow_count;//允许登录次数3-10次
        int flag;//是否开启 0:不开启; 1:开启
        int sendmail;//是否发送邮件 0:否; 1:是
        int light_flag;//是否闪光灯 0:否; 1:是
        int light_hold;//闪光灯持续时间10-300s
        int output_flag;//是否外部输出 0:不输出 1:输出
        int output_hold;//外部输出持续时间10-300s
};


struct SGP_EMAIL_INFO
{
        int alarm;//是否使用报警邮件 0:否;1:是
        int alarm_value;//报警邮件间隔 1-3600s
        int enclosure;//是否带附件 0:否; 1:是
        int encry_type;//加密方式 0:none; 1:tls; 2:ssl
        char from[STRING_LENGH];//发件人
        int health;//是否使用健康邮件 0:否; 1:是
        int health_value;//健康邮件间隔 1-3600s
        int is_anon;//是否匿名 0:否; 1:是
        char password[STRING_LENGH];//登录密码，密文传输
        int smtp_port;//smtp服务端口，默认25
        char smtp_server[STRING_LENGH];//smtp服务器,默认空xxx.xxx.xxx.xxx
        char subject[STRING_LENGH];//主题
        char username[STRING_LENGH];//登录服务器用户名
        int mailto_num;//收件人数量
        char mailto[5][STRING_LENGH];//收件人列表
};

struct SGP_FILL_LIGHT_INFO
{
        int brightness;/*亮度 0-100,
                                   0 - 20一档
                                   21 - 40二档
                                   41 - 60三档
                                   61 - 80四档
                                   81 - 100五档*/
        int light;//灯开启状态 0:关闭; 1:开启
        int mode;//灯模式 0:手动; 1:自动
};

struct SGP_CONFIG
{
        int type;//报警类型 1:高温报警; 2:低温报警; 3:平均温报警; 4:高低温报警
				 //5:温升报警 6:温差报警(web2.0增加)
        int condition;//条件 1:高于; 2:低于 3:匹配;
        float high_temp;//配置高温
        float low_temp;//配置低温
        float avg_temp;//配置平均温
        int objtype;//类型 0:冷热点; 1:点; 2:线; 3:矩形; 4:多边形; 5:圆形
        SGP_POINT points[7];
};

struct SGP_TEMPALARMNOTIFY
{
        char vl_image_url[200];//报警记录可见光截图地址
        char vl_video_url[200];//可见光视频地址
        char ir_image_url[200];//报警记录红外截图地址
        char ir_video_url[200];//红外视频地址
        float high_temp;//高温温度,高温报警时有效
        float low_temp;//低温温度,低温报警时有效
        float avg_temp;//平均温度,平均温报警时有效
        int temp_flag;//报警类型，0代表平均温，1代表高温报警，2代表低温报警，3代表高低温报警(web2.0 不支持)
					  //5温升报警 6温差报警(web2.0增加)
        int type;//1:温度报警; 2:热点报警; 3:冷点报警(web2.0 不支持)
        char name[STRING_LENGH];//名称
        char time[STRING_LENGH];//报警时间，格式为2020-05-21 12:22:33
        SGP_CONFIG config;//配置
};

struct SGP_OBJTEMPALARMNOTIFY
{
	char vl_image_url[200];//报警记录可见光截图地址
	char vl_video_url[200];//可见光视频地址
	char ir_image_url[200];//报警记录红外截图地址
	char ir_video_url[200];//红外视频地址

	char obj_name1[STRING_LENGH];//分析对象1的名称
	char obj_name2[STRING_LENGH];//分析对象2的名称
	float fTemp1;//分析对象1的温度
	float fTemp2;//分析对象2的温度
	float fTempDiff;//分析对象1、2的温差值
	float fTempThreshold;//分析对象1、2的温差阈值
	int iTempFlag;//判断条件： 0:温差大于阈值; 1:温差小于阈值
	float iTempType1;//分析对象1的温度类型，0: 最高温，1:最低温, 2:平均温
	float iTempType2;//分析对象2的温度类型，0: 最高温，1:最低温, 2:平均温
	char name[STRING_LENGH];//名称
	char time[STRING_LENGH];//报警时间，格式为2020-05-21 12:22:33
};

struct SGP_MEMORYFULLNOTIFY
{
        int total;//总存储，单位M
        int free;//可用大小，单位M
        int limit;//报警阈值，可用小于报警阈值时报警，单位M
};

struct SGP_ACCESSVIOLATIONNOTIFY
{
        char user[STRING_LENGH];//异常登录用户
        char ip[STRING_LENGH];//异常登录IP
        char time[STRING_LENGH];//异常登录时间
};

struct SGP_NETWORKERRORNOTIFY
{
        int type;//类型0:网络中断 1:ip冲突; 2:ping不通网关，ip为网关
        char ip[STRING_LENGH];//ip地址
};

struct SGP_ALARMINPUTNOTIFY
{
        char time[STRING_LENGH];//报警时间，格式为2020-05-21 12:22:33
        char vl_image_url[STRING_LENGH];//报警记录可见光截图，http jpeg路径
        char ir_image_url[STRING_LENGH];//报警记录红外截图，http jpeg路径
        char vl_image_content[STRING_LENGH];//可见光JPEG图片得BASE64格式
        char ir_image_content[STRING_LENGH];//红外JPEG图片得BASE64格式
        char vl_video_url[STRING_LENGH];//可见光录像地址
        char ir_video_url[STRING_LENGH];//红外录像地址
};

struct SGP_IR_IMAGE_INFO
{
	int denoise_flag_2d;//2D降噪开关
	int denoise_value_2d;//2D降噪值
	int denoise_flag_3d;//3D降噪开关
	int denoise_value_3d;//3D降噪值
};

enum SGP_IR_IMAGE_INFO_ENUM
{
	SGP_2D_FLAG = 1,//2D降噪开关
	SGP_2D_VALUE = 2,//2D降噪值
	SGP_3D_FLAG = 3,//3D降噪开关
	SGP_3D_VALUE = 4,//3D降噪值
	SGP_SAVE_IR_IMAGE_INFO = 5,//保存参数信息
};

struct SGP_FIRE_ALARM
{
	float high_temp;//高温温度,高温报警时有效
	float low_temp;//低温温度，低温报警时有效
	float avg_temp;//平均温度，平均温报警时有效
	char time[STRING_LENGH];//报警时间，格式为2020-05-21 12:22:33
	char capture_time[STRING_LENGH];//报警抓图时间，格式为2020-05-21 12:22:33
	char vl_image_url[STRING_LENGH];//报警记录可见光截图，http jpeg路径
	char ir_image_url[STRING_LENGH];//报警记录红外截图，http jpeg路径
	char vl_video_url[STRING_LENGH];//可见光录像地址
	char ir_video_url[STRING_LENGH];//红外录像地址
};

struct SGP_MEASURE_TEMP_INFO
{
	float jwtemp;//焦温温度
	float realshuttertemp;//实时快门温度
	float lastshuttertemp;//上次快门温度
	float realmirrortemp;//实时镜筒温度
	int	jwgears;//焦温档位
	int devgain;//探测器参数Gain
	int devint;//探测器参数Int
	int devres;//探测器参数Res
	int devgsk;//探测器参数gsk
	float centertemp;//全图中心温度
	float centermaxtemp;//全图最高温度
	float centermintemp;//全图最低温度
	int	centerx16;//中心点X16
	int centery16;//中心点Y16
	int avgshutter;//快门本底均值
	int rasel;//探测器参数RASEL
	int hssd;//探测器参数HSSD
	int gsktestnum;//探测器参数gsk_test_num
	int gskval;//探测器参数gsk_val
	bool tempStabilityState;//焦温波动判断设备稳定性
	float sharpnessleftup;//图像内左上区域的清晰度
	float sharpnessrightup;//图像内右上区域的清晰度
	float sharpnesscenter;//图像内中间区域的清晰度
	float sharpnessleftdown;//图像内左下区域的清晰度
	float sharpnessrightdown;//图像内右下区域的清晰度
};

struct MeasureParam
{
	int lens;												//镜头类型
	short sY16Offset;										//Y16偏移量(arm)
	int nKF;												//查曲线时Y16的缩放量(定点化100倍,1~100,默认值为100)(arm)
	int nB1;												//查曲线时Y16的偏移量(定点化100倍)(arm)

	int nDistance_a0;										//距离校正系数(定点化10000000000倍)(arm)
	int nDistance_a1;										//距离校正系数(定点化1000000000倍)(arm)
	int nDistance_a2;										//距离校正系数(定点化10000000倍)(arm)
	int nDistance_a3;										//距离校正系数(定点化100000倍)(arm)
	int nDistance_a4;										//距离校正系数(定点化100000倍)(arm)
	int nDistance_a5;										//距离校正系数(定点化10000倍)(arm)
	int nDistance_a6;										//距离校正系数(定点化1000倍)(arm)
	int nDistance_a7;										//距离校正系数(定点化100倍)(arm)
	int nDistance_a8;										//距离校正系数(定点化100倍)(arm)

	int nK1;												//快门温漂系数(定点化100倍)(arm)
	int nK2;												//镜筒温漂系数(定点化100倍)(arm)
	int nK3;												//环温修正系数(定点化10000倍)(arm)
	int nB2;												//环温修正偏移量(定点化10000倍)(arm)

	int nKFOffset;											//自动校温KF偏移量，置零后恢复出厂校温设置(arm)
	int nB1Offset;											//自动校温B1偏移量，置零后恢复出厂校温设置(arm)

	int nAtmosphereTransmittance;							//大气透过率(定点化100倍，范围0~100)(arm)
	int nHumidity;                                          //湿度(定点化100倍，默认60)

	float fEmiss;											//发射率(arm)
	float fDistance;										//测温距离(arm)
	float fReflectT;										//背景温度(参数行)
	float fAmbient;											//环境温度(参数行)
	float fWindowTransmittance;								//窗口透过率(范围0~1)(arm)
	float fWindowTemperature;								//窗口温度(参数行)

	bool bShutterCorrection;								//快门修正开关(arm)
	bool bLensCorrection;									//镜筒修正开关(arm)
	bool bEmissCorrection;									//发射率修正开关(arm)
	bool bDistanceCorrection;								//距离修正开关(arm)
	bool bAmbientCorrection;								//环温修正开关(arm)
	bool bB1Correction;										//B1修正开关(arm)
	bool bAtmosphereCorrection;								//大气透过率修正开关(arm)
	bool bWindowTransmittanceCorrection;					//窗口透过率开关(arm)
	bool bHumidityCorrection;                               //湿度修正开关(arm)
};

struct SGP_FILE_TEMP_INFO
{
	int width;	//y16的宽,红外模组宽
	int height; //y16的高，红外模组高
	MeasureParam tempParam; // 测温参数
	char *curve; // 测温曲线
	int curveLens; // 长度
	short *y16AndParaLineInfo; // y16和参数行信息
	int length;		   // y16的参数行对应的字节数
};

/**
* @brief        温度告警回调函数
* @param
* notify        输出参数
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_TEMPALARMCALLBACK)(SGP_TEMPALARMNOTIFY notify, void *pUser);

/**
* @brief        对象温差告警回调函数
* @param
* notify        输出参数
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_OBJTEMPALARMCALLBACK)(SGP_OBJTEMPALARMNOTIFY notify, void *pUser);

/**
* @brief        内存已满回调函数
* @param
* notify        输出参数
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_MEMORYFULLCALLBACK)(SGP_MEMORYFULLNOTIFY notify, void *pUser);

/**
* @brief        存储故障回调函数
* @param
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_STORAGEERRORCALLBACK)(void *pUser);

/**
* @brief        推流异常回调函数
* @param
* type			输出参数
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_RTSPERRORCALLBACK)(int type, void *pUser);

/**
* @brief        非法访问回调函数
* @param
* notify        输出参数
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_ACCESSVIOLATIONCALLBACK)(SGP_ACCESSVIOLATIONNOTIFY notify, void *pUser);

/**
* @brief        网络异常回调函数
* @param
* notify        输出参数
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_NETWORKERRORCALLBACK)(SGP_NETWORKERRORNOTIFY notify, void *pUser);

/**
* @brief        图像数据回调函数
* @param
* outdata       输出参数，图像数据
* w				输出参数，图像数据宽
* h				输出参数，图像数据高
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_RTSPCALLBACK)(unsigned char *outdata, int w, int h, void *pUser);

/**
* @brief        本地录像回调函数
* @param
* state         输出参数,1：开始录制 2：录制中 3：停止录制
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_RECORDCALLBACK)(int state, void *pUser);

/**
* @brief        温度矩阵回调函数
* @param
* temp			输出参数，温度矩阵数据
* w				输出参数，温度矩阵宽
* h				输出参数，温度矩阵高
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_TEMPCALLBACK)(short *temp, int w, int h, void *pUser);

/**
* @brief        告警输入回调函数
* @param
* notify		输出参数，告警信息
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_ALARMINPUTCALLBACK)(SGP_ALARMINPUTNOTIFY notify, void *pUser);

/**
* @brief        火灾报警回调函数
* @param
* notify		输出参数，告警信息
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_FIREALARMCALLBACK)(SGP_FIRE_ALARM notify, void *pUser);

/**
* @brief        自动调焦回调函数
* @param
* result		输出参数，0:调焦完成，结果不清晰 1:调焦完成，结果清晰
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_AUTOFOCUSCALLBACK)(int result, void *pUser);

/**
* @brief        Y16回调函数
* @param
* temp			输出参数，Y16和参数行数据
* length		输出参数，Y16和参数行数据的长度
* pUser			输出参数
* @return       无
*/
typedef void(*SGP_Y16CALLBACK)(short *y16, int length, void *pUser);

