#pragma once

#include <ros/ros.h>
#include <sys/time.h>
#include <string.h>
#include <vector>
#include <unistd.h>
#include <ctime>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
// opencv
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
// 海康相机
#include "HCNetSDK.h"
#include "PlayM4.h"

using namespace std;

// Camera类
class Camera
{
public:
    static Camera * pThis;  //静态对象指针

private:
	bool m_isLoop;					//是否循环
    // ros
    ros::NodeHandle* m_nh;          //本节点句柄
    string m_img_topic;             //图像发布话题名
    //图像属性
    int m_width;					//图像宽度
	int m_height;					//图像高度
	int m_frameSize;				//图像大小
	//相机参数
	int m_connectTime;						//连接时间
	int m_reconnectTime;					//重连时间
    //登录信息
	NET_DVR_USER_LOGIN_INFO m_LoginInfo;	//登录信息
	NET_DVR_DEVICEINFO_V40 m_DeviceInfo;	//设备信息
	NET_DVR_PREVIEWINFO m_PlayInfo;			//预览信息
	long m_userID;							//登录后的用户ID
	long m_handle;							//预览回调结果
	LONG m_nPort;

public:
	//构造、析构函数
	Camera(ros::NodeHandle* nh);
	~Camera();
    //参数加载
    void LoadParam();
    //接收循环，Receive Loop
    void ReceivingLoop();
    //回调函数
	static void CALLBACK g_RealDataCallBack_V30(LONG IRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* pUser);
	static void CALLBACK DecCBFun(int nPort, char* pBuf, int nSize, FRAME_INFO* pFrameInfo, void* nUser, int nReserved2);

private:
	//内存释放
	void Dump();
	//相机初始化
	bool CameraInit();
	//相机登录
	bool CameraLogin();
};