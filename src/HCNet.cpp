#include "HCNet.h"


//全局变量
Camera *Camera::pThis = NULL;

//数据解码回调函数，
//功能：将YV_12格式的视频数据流转码为可供opencv处理的BGR类型的图片数据
void CALLBACK Camera::DecCBFun(int nPort, char* pBuf, int nSize, FRAME_INFO* pFrameInfo, void* nUser, int nReserved2)
{	
    // 话题发布
    static image_transport::ImageTransport it(*(pThis->m_nh));
    static image_transport::Publisher pubImg = it.advertise(pThis->m_img_topic, 1);
	cv::Mat BGRImage;
	if (pFrameInfo->nType == T_YV12)
	{
		if (BGRImage.empty())
		{
			BGRImage.create(pFrameInfo->nHeight, pFrameInfo->nWidth, CV_8UC3);
		}
		cv::Mat YUVImage(pFrameInfo->nHeight + pFrameInfo->nHeight / 2, pFrameInfo->nWidth, CV_8UC1, (unsigned char*)pBuf);
		cv::Mat resizeImage;
		cvtColor(YUVImage, BGRImage, cv::COLOR_YUV2BGR_YV12);
		resize(BGRImage, resizeImage, cv::Size(pThis->m_width, pThis->m_height));

		// 发布图像
	    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", resizeImage).toImageMsg();
	    pubImg.publish(msg);

		YUVImage.~Mat();
		BGRImage.~Mat();
		resizeImage.~Mat();
	}
}


void CALLBACK Camera::g_RealDataCallBack_V30(LONG IRealHandle, DWORD dwDataType, BYTE *pBuffer, DWORD dwBufSize, void* pUser)
{
	DWORD dRet = 0;
	BOOL inData = FALSE;

	if(pThis == NULL)
		return;

	switch (dwDataType)
	{
		// 系统头
		case NET_DVR_SYSHEAD:
			// 获取播放库未使用的通道号
			if (!PlayM4_GetPort(&pThis->m_nPort))
			{
				break;
			}
			// 
			if (!PlayM4_OpenStream(pThis->m_nPort, pBuffer, dwBufSize, pThis->m_frameSize))
			{
				dRet = PlayM4_GetLastError(pThis->m_nPort);
				break;
			}
			// 设置解码回调函数 仅仅解码不显示
			if (!PlayM4_SetDecCallBack(pThis->m_nPort, pThis->DecCBFun))
			{
				dRet = PlayM4_GetLastError(pThis->m_nPort);
				break;
			}
			// 打开视频解码
			if (!PlayM4_Play(pThis->m_nPort, NULL))
			{
				dRet = PlayM4_GetLastError(pThis->m_nPort);
				break;
			}
		// 码流数据
		case NET_DVR_STREAMDATA:
			if (dwBufSize > 0 && pThis->m_nPort != -1)
			{
				inData = PlayM4_InputData(pThis->m_nPort, pBuffer, dwBufSize);
				while (!inData)
				{
					inData = PlayM4_InputData(pThis->m_nPort, pBuffer, dwBufSize);
					printf("PlayM4_InputData faild %d\n", PlayM4_GetLastError(pThis->m_nPort));
				}
			}
			break;
		default:
			inData = PlayM4_InputData(pThis->m_nPort, pBuffer, dwBufSize);
			while (!inData)
			{
				inData = PlayM4_InputData(pThis->m_nPort, pBuffer, dwBufSize);
				printf("PlayM4_InputData failed 2\n");
			}
			break;
	}
}


// 构造函数，将相机参数设置成配置文件中的参数
Camera::Camera(ros::NodeHandle* nh):m_nh(nh), m_isLoop(true), m_userID(-1), m_handle(-1)
{
    pThis = this;
    // 参数加载
    LoadParam();
}

// 加载参数
void Camera::LoadParam()
{
	// 相机登录参数
    m_nh->param<int>("camera/connect_time", m_connectTime, 2000);
    m_nh->param<int>("camera/reconnect_time", m_reconnectTime, 10000);
	m_DeviceInfo = { 0 };
	m_LoginInfo = { 0 };
	m_PlayInfo = { 0 };
    string device_address_, user_name_, password_;
    int asyn_login, wport;
    m_nh->param<string>("camera/device_address", device_address_, "192.168.1.71");
    m_nh->param<string>("camera/user_name", user_name_, "admin");
    m_nh->param<string>("camera/password", password_, "123456");
    const char * device_address = device_address_.c_str();
	const char * user_name = user_name_.c_str();
	const char * password = password_.c_str();
    m_nh->param<int>("camera/asyn_login", asyn_login, 0);
    m_nh->param<int>("camera/port", wport, 8000);
    m_LoginInfo.bUseAsynLogin = asyn_login;                                 // 同步登录方式
    m_LoginInfo.wPort = wport;                                              // 设备服务端口
	strcpy(m_LoginInfo.sDeviceAddress, device_address); 				    // 设备IP地址
	strcpy(m_LoginInfo.sUserName, user_name); 							    // 设备登录用户名
	strcpy(m_LoginInfo.sPassword, password);							    // 设备登录密码
	// 默认图像属性
    m_nh->param<int>("camera/frame_width", m_width, 1920);
    m_nh->param<int>("camera/frame_height", m_height, 1080);
    m_frameSize = m_width * m_height;
    // 话题名
    m_nh->param<string>("topic/img_topic_name", m_img_topic, "hcnet_img");
}


//析构函数
Camera::~Camera()
{
	Dump();
}


// 内存释放
void Camera::Dump()
{
	//内存释放，Mat类自动管理
	//相机相关释放
	NET_DVR_StopRealPlay(m_handle);			//关闭预览
	NET_DVR_Logout(m_userID);				//注销用户
	NET_DVR_Cleanup();						//释放SDK资源
}


// 相机初始化
bool Camera::CameraInit()
{
    cout << "[INFO]：进行相机初始化" << endl;
    if (NET_DVR_Init())
        return true;
	else
		return false;
}


// 登录函数
bool Camera::CameraLogin()
{
	// step.1---------设置连接与重连时间-----------------//
	NET_DVR_SetConnectTime(m_connectTime, 1);
	NET_DVR_SetReconnect(m_reconnectTime, true);
	// step.2---------登录----------------------------//
    cout << "[INFO]：登录相机" << endl;
    m_userID = NET_DVR_Login_V40(&m_LoginInfo, &m_DeviceInfo);
    if (m_userID < 0)
		return false;
	else
		return true;
}


// 循环接收相机数据
void Camera::ReceivingLoop()
{
	// step.1---------相机初始化-----------------------//
    if(!CameraInit())
    {
        cout << "[ERROR]：相机初始化失败!" << endl;
        m_isLoop = false;
		return;
	}
	// step.2---------相机登陆-------------------------//
    if(!CameraLogin())
    {

        cout << "[ERROR]：相机登陆失败！" << endl;
        m_isLoop = false;
        return;
	}
	// step.3---------相机预览设置---------------------//
	m_PlayInfo.hPlayWnd = NULL; 	// 窗口为空，设备SDK不解码只取流
	m_PlayInfo.lChannel = 1; 		// Channel number 设备通道
	m_PlayInfo.dwStreamType = 0;	// 码流类型，0-主码流，1-子码流，2-码流3，3-码流4, 4-码流5,5-码流6,7-码流7,8-码流8,9-码流9,10-码流10
	m_PlayInfo.dwLinkMode = 0;		// 0：TCP方式,1：UDP方式,2：多播方式,3 - RTP方式，4-RTP/RTSP,5-RSTP/HTTP
	m_PlayInfo.bBlocked = 1; 		// 0-非阻塞取流, 1-阻塞取流, 如果阻塞SDK内部connect失败将会有5s的超时才能够返回,不适合于轮询取流操作.
	// step.4---------实时预览回调---------------------//
	m_handle = NET_DVR_RealPlay_V40(m_userID, &m_PlayInfo, g_RealDataCallBack_V30, NULL);
	// step.5---------设置循环------------------------//
	while (m_isLoop)
	{
		continue;
	}
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "hcnet_sdk");
    ros::NodeHandle nh;
    // 创建camera对象
    Camera m_camera(&nh);
    // 循环接收图像
    m_camera.ReceivingLoop();

    // ros::spin();
    return 0;
}