#include <iostream>
#include <fstream>
#include <string>



// DLT-664格式热图结构体
struct IptPictureFrame
{
    char *IRImage;                  // 红外视频截图数据文件
    unsigned char FileVersion[2];   // 文件版本
    unsigned short Width;           // 温度点阵宽度
    unsigned short Height;          // 温度点阵高度
    unsigned char DateTime[14];     // 拍摄时间
    float *IRData;                  // 红外温度值点阵数据
    float Emiss;                    // 辐射率
    float AmbientTemperature;       // 环境温度
    unsigned char Len;              // 镜头度数
    unsigned int Distance;          // 拍摄距离
    unsigned char RelativeHumidity; // 相对湿度
    float ReflectiveTemperature;    // 反射温度
    unsigned char Productor[32];    // 生产厂家
    unsigned char Type[32];         // 产品型号
    unsigned char SerialNO[32];     // 产品序列号
    double Longitude;               // 经度
    double Latitude;                // 纬度
    int Altitude;                   // 海拔
    unsigned int DescriptionLength; // 备注信息长度
    unsigned char *DescriptionData; // 备注信息,分析对象等信息
    unsigned int IRDataOffset;      // 红外数据的起始偏移地址
    unsigned char FileEndType[16];  // 文件末尾标识
public:
    IptPictureFrame()
    {
        memset(this, 0, sizeof(IptPictureFrame));
    }
};

// FIR格式热图结构体
struct FirFrame
{
    unsigned char FileFlag[4];  // 文件标记
    float OptiTrans;            // 光学透过率
    float Emiss;                // 辐射率
    float Distance;             // 拍摄距离
    float AmbientTemperature;   // 环境温度
    float RelativeHumidity;     // 相对湿度
    unsigned short Width;       // 温度点阵宽度
    unsigned short Height;      // 温度点阵高度
    unsigned char Precision[1]; // 精度
    short *IRData;              // 红外温度值点阵数据
public:
    FirFrame()
    {
        memset(this, 0, sizeof(FirFrame));
    }
};

// imgPath值类似E://20210506//20210506152031609temp.jpg
// DLT-664格式热图
static int GetGWTempEx(std::string imgPath, FLOAT_T &maxTemp, FLOAT_T &minTemp, FLOAT_T &avgTemp)
{
    IptPictureFrame pictureFrame;
    int readByes = 0;
    unsigned char expectedFileEndType[16] = {0x37, 0x66, 0x07, 0x1a, 0x12, 0x3a, 0x4c, 0x9f, 0xa9, 0x5d, 0x21, 0xd2, 0xda, 0x7d, 0x26, 0xbc};
    ifstream inFile(imgPath.c_str(), ios::in | ios::binary);
    if (!inFile)
    {
        cout << "error" << endl;
        return 0;
    }
    inFile.seekg(0, ios::end);
    char singleChar;
    int picLen = inFile.tellg();
    while (!inFile.eof())
    {
        inFile.read((char *)&singleChar, sizeof(char));
        picLen++;
        cout << "还没有到文件尾"
             << "当前读取的字符是" << singleChar << endl;
    }

    if (inFile.eof())
        cout << "到文件尾了" << endl;
    picLen = picLen - 1;
    cout << "Picture length is " << picLen << endl;
    inFile.clear();
    inFile.seekg(picLen - 16, ios::beg);
    inFile.read((char *)&pictureFrame.FileEndType, sizeof(pictureFrame.FileEndType));
    readByes = inFile.gcount();
    cout << "读取的字节数" << readByes << endl;
    // 打印文件结尾标志
    int wrongFileEndNum = 0;
    for (int i = 0; i < 16; i++)
    {
        cout << hex << (int)pictureFrame.FileEndType[i] << endl;
        if (pictureFrame.FileEndType[i] != expectedFileEndType[i])
            wrongFileEndNum++;
    }
    if (wrongFileEndNum == 0)
    {
        printf("文件结尾字符数组正确\n");
    }
    else
    {
        printf("文件结尾字符数组错误\n");
    }
    // 获取红外数据的起始偏移地址
    inFile.clear();
    inFile.seekg(picLen - 20, ios::beg);
    inFile.read((char *)&pictureFrame.IRDataOffset, sizeof(pictureFrame.IRDataOffset));
    cout << "获取红外数据的起始偏移地址是：" << pictureFrame.IRDataOffset << endl;

    // 获取文件版本
    inFile.clear();
    inFile.seekg(pictureFrame.IRDataOffset, ios::beg);
    inFile.read((char *)&pictureFrame.FileVersion, sizeof(pictureFrame.FileVersion));
    for (int i = 0; i < 2; i++)
    {
        cout << hex << (int)pictureFrame.FileVersion[i] << endl;
    }
    char cFileVersion[2];
    ft::convertUnCharToStr(cFileVersion, pictureFrame.FileVersion, 2);
    printf("文件版本是%s\n", cFileVersion);
    // 获取图像文件宽
    inFile.read((char *)&pictureFrame.Width, sizeof(pictureFrame.Width));
    cout << dec << "获取红外文件的宽是：" << pictureFrame.Width << endl;
    // 获取图像文件高
    inFile.read((char *)&pictureFrame.Height, sizeof(pictureFrame.Height));
    cout << dec << "获取红外文件的高是：" << pictureFrame.Height << endl;
    // 获取图片拍摄时间
    inFile.read((char *)&pictureFrame.DateTime, sizeof(pictureFrame.DateTime));
    /*for (int i = 0; i < 14; i++)
    {
    cout << dec << (char)pictureFrame.DateTime[i] << endl;
    }*/
    printf("图片拍摄时间是：%s\n", pictureFrame.DateTime);
    // 获取图片温度矩阵信息
    FLOAT_T max = 0, min = 0, sum = 0, avg = 0;
    printf("温度矩阵宽是%d,高是%d\n", pictureFrame.Width, pictureFrame.Height);
    pictureFrame.IRData = (FLOAT_T *)calloc(pictureFrame.Width * pictureFrame.Height, sizeof(FLOAT_T));
    inFile.read((char *)&pictureFrame.IRData[0], sizeof(FLOAT_T));

    max = pictureFrame.IRData[0], min = pictureFrame.IRData[0];
    for (INT32_T i = 1; i < pictureFrame.Width * pictureFrame.Height; i++)
    {
        inFile.read((char *)&pictureFrame.IRData[i], sizeof(FLOAT_T));
        // printf("第%d个点的温度是%f\n", i, matrix[i]);
        if (pictureFrame.IRData[i] >= max)
        {
            max = pictureFrame.IRData[i];
        }

        if (min >= pictureFrame.IRData[i])
        {
            min = pictureFrame.IRData[i];
        }
        sum = sum + pictureFrame.IRData[i];
    }
    maxTemp = max;
    minTemp = min;
    avg = sum / (pictureFrame.Width * pictureFrame.Height);
    avgTemp = avg;

    printf("最高温是%f,最低温是%f,平均温%f\n", max, min, avg);

    // 获取辐射率
    inFile.read((char *)&pictureFrame.Emiss, sizeof(pictureFrame.Emiss));
    printf("获取的辐射率是%f\n", pictureFrame.Emiss);

    // 获取环境温度
    inFile.read((char *)&pictureFrame.AmbientTemperature, sizeof(pictureFrame.AmbientTemperature));
    printf("获取的环境温度是%f\n", pictureFrame.AmbientTemperature);

    // 获取镜头度数
    inFile.read((char *)&pictureFrame.Len, sizeof(pictureFrame.Len));
    printf("获取的镜头度数是%s\n", pictureFrame.Len);

    // 获取拍摄距离
    inFile.read((char *)&pictureFrame.Distance, sizeof(pictureFrame.Distance));
    printf("获取的拍摄距离是%u\n", pictureFrame.Distance);

    // 获取相对湿度
    inFile.read((char *)&pictureFrame.RelativeHumidity, sizeof(pictureFrame.RelativeHumidity));
    printf("获取的相对湿度是%d\n", (int)pictureFrame.RelativeHumidity);

    // 获取反射温度
    inFile.read((char *)&pictureFrame.ReflectiveTemperature, sizeof(pictureFrame.ReflectiveTemperature));
    printf("获取的反射温度是%f\n", pictureFrame.ReflectiveTemperature);

    // 获取生产厂家
    inFile.read((char *)&pictureFrame.Productor, sizeof(pictureFrame.Productor));
    printf("获取的生产厂家是%s\n", pictureFrame.Productor);

    // 获取产品型号
    inFile.read((char *)&pictureFrame.Type, sizeof(pictureFrame.Type));
    printf("获取的产品型号是%s\n", pictureFrame.Type);

    // 获取产品序列号
    inFile.read((char *)&pictureFrame.SerialNO, sizeof(pictureFrame.SerialNO));
    printf("获取的产品序列号是%s\n", pictureFrame.SerialNO);

    // 获取经度
    inFile.read((char *)&pictureFrame.Longitude, sizeof(pictureFrame.Longitude));
    printf("获取的经度是%f\n", pictureFrame.Longitude);

    // 获取纬度
    inFile.read((char *)&pictureFrame.Latitude, sizeof(pictureFrame.Latitude));
    printf("获取的纬度是%f\n", pictureFrame.Latitude);

    // 获取海拔
    inFile.read((char *)&pictureFrame.Altitude, sizeof(pictureFrame.Altitude));
    printf("获取的海拔是%d\n", pictureFrame.Altitude);

    // 获取备注信息长度
    inFile.read((char *)&pictureFrame.DescriptionLength, sizeof(pictureFrame.DescriptionLength));
    printf("获取的备注信息长度是%u\n", pictureFrame.DescriptionLength);

    // 清温度矩阵内存
    if (pictureFrame.IRData)
    {
        free(pictureFrame.IRData);
        pictureFrame.IRData = GUIDEIR_NULL;
    }

    // 关闭文件
    inFile.close();

    // 比较温度
    float leftRange = 1, rightRange = 60;
    // 比较最高温
    if (max > avg && fabs(max - avg) > 1e-6 && max < rightRange && fabs(max - rightRange) > 1e-6 && max > leftRange && fabs(max - leftRange) > 1e-6)
        li::logAndPrint("Success ! 最高温" + ft::ConvertFloattoStr(max) + "在" + ft::ConvertFloattoStr(leftRange) + "和" + ft::ConvertFloattoStr(rightRange) + "之间");
    else
        li::logAndPrint("Fail ! 最高温" + ft::ConvertFloattoStr(max) + "不在" + ft::ConvertFloattoStr(leftRange) + "和" + ft::ConvertFloattoStr(rightRange) + "之间");
    // 比较最低温
    if (min > leftRange && fabs(min - leftRange) > 1e-6 && min < rightRange && fabs(min - rightRange) > 1e-6 && min < avg && fabs(min - avg) > 1e-6)
        // if ((int)lowTemp > leftRange && (int)lowTemp < rightRange )
        li::logAndPrint("Success ! 最低温" + ft::ConvertFloattoStr(min) + "在" + ft::ConvertFloattoStr(leftRange) + "和" + ft::ConvertFloattoStr(rightRange) + "之间");
    else
        li::logAndPrint("Fail ! 最低温" + ft::ConvertFloattoStr(min) + "不在" + ft::ConvertFloattoStr(leftRange) + "和" + ft::ConvertFloattoStr(rightRange) + "之间");
    // 比较平均温
    if (avg > leftRange && fabs(avg - leftRange) > 1e-6 && avgTemp < max && fabs(avgTemp - max) > 1e-6 && avg > min && fabs(avg - min) > 1e-6)
        li::logAndPrint("Success ! 平均温" + ft::ConvertFloattoStr(avg) + "在" + ft::ConvertFloattoStr(leftRange) + "和" + ft::ConvertFloattoStr(rightRange) + "之间");
    else
        li::logAndPrint("Fail ! 平均温" + ft::ConvertFloattoStr(avg) + "不在" + ft::ConvertFloattoStr(leftRange) + "和" + ft::ConvertFloattoStr(rightRange) + "之间");

    return 0;
}

// imgPath值类似E://20210506//20210506152031609temp.jpg
// FIR热图格式
static int GetFirFile(std::string imgPath)
{
    FirFrame pictureFrame;
    int readByes = 0;
    ifstream inFile(imgPath.c_str(), ios::in | ios::binary);
    if (!inFile)
    {
        cout << "error" << endl;
        return 0;
    }
    inFile.seekg(0, ios::beg);
    inFile.read((char *)&pictureFrame.FileFlag, sizeof(pictureFrame.FileFlag));
    printf("文件标记是%s\n", &pictureFrame.FileFlag);
    // 获取光学透过率
    inFile.read((char *)&pictureFrame.OptiTrans, sizeof(pictureFrame.OptiTrans));
    cout << dec << "获取的光学透过率是：" << pictureFrame.OptiTrans << endl;

    // 获取辐射率
    inFile.read((char *)&pictureFrame.Emiss, sizeof(pictureFrame.Emiss));
    cout << dec << "获取的辐射率是：" << pictureFrame.Emiss << endl;

    // 获取拍摄距离
    inFile.read((char *)&pictureFrame.Distance, sizeof(pictureFrame.Distance));
    printf("拍摄距离是：%f\n", pictureFrame.Distance);

    // 获取环境温度
    inFile.read((char *)&pictureFrame.AmbientTemperature, sizeof(pictureFrame.AmbientTemperature));
    printf("获取的环境温度是%f\n", pictureFrame.AmbientTemperature);

    // 获取相对湿度
    inFile.read((char *)&pictureFrame.RelativeHumidity, sizeof(pictureFrame.RelativeHumidity));
    printf("获取的相对湿度是%f\n", pictureFrame.RelativeHumidity);

    // 获取图像文件高
    inFile.read((char *)&pictureFrame.Height, sizeof(pictureFrame.Height));
    cout << dec << "获取文件的高是：" << pictureFrame.Height << endl;

    // 获取图像文件宽
    inFile.read((char *)&pictureFrame.Width, sizeof(pictureFrame.Width));
    cout << dec << "获取文件的宽是：" << pictureFrame.Width << endl;

    // 获取精度
    inFile.read((char *)&pictureFrame.Precision, sizeof(pictureFrame.Precision));
    std::string preciStr = ft::byteArrayToString(pictureFrame.Precision, 1);
    printf("获取的精度是%s\n", preciStr.c_str());

    inFile.seekg(64, ios::beg);
    // 获取图片温度矩阵信息
    short max = 0, min = 0, sum = 0;
    printf("温度矩阵宽是%d,高是%d\n", pictureFrame.Width, pictureFrame.Height);
    pictureFrame.IRData = (short *)calloc(pictureFrame.Width * pictureFrame.Height, sizeof(short));
    inFile.read((char *)&pictureFrame.IRData[0], sizeof(short));

    max = pictureFrame.IRData[0], min = pictureFrame.IRData[0];
    for (INT32_T i = 1; i < pictureFrame.Width * pictureFrame.Height; i++)
    {
        inFile.read((char *)&pictureFrame.IRData[i], sizeof(short));
        // printf("第%d个点的温度是%d\n", i, &pictureFrame.IRData[i]);
        if (pictureFrame.IRData[i] >= max)
        {
            max = pictureFrame.IRData[i];
        }

        if (min >= pictureFrame.IRData[i])
        {
            min = pictureFrame.IRData[i];
        }
        sum = sum + pictureFrame.IRData[i];
    }
    printf("最高温是%hd,最低温是%hd\n", max, min);

    // 清温度矩阵内存
    if (pictureFrame.IRData)
    {
        free(pictureFrame.IRData);
        pictureFrame.IRData = GUIDEIR_NULL;
    }

    // 关闭文件
    inFile.close();

    return 0;
}

int main()
{
    std::string imgPath = "/home/yuan/server_ws/src/thermal_ds4025ft/scripts/HeatMap.jpg";

    GetFirFile(imgPath);

    return 0;
}