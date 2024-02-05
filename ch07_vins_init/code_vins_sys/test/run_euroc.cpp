
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>

#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/cpsu-zs/work/docker/slam/data/EuRoC/MH_05_difficult/mav0/";
string sConfig_path = "../config/";

std::shared_ptr<System> pSystem;

// 读取IMU数据线程函数
void PubImuData()
{
	string sImu_data_file = sConfig_path + "MH_05_imu0.txt";
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);
		// IMU数据打包成IMU_MSG结构体，按照当前读取到的文本数据逐一赋值
		ssImuData >> dStampNSec >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec / 1e9, vGyr, vAcc); // 时间戳规划成秒级
		usleep(5000*nDelayTimes); // 按照时间戳的间隔，延时一定时间，模拟实时性 5000*2 μs = 10ms
	}
	fsImu.close();
}

void PubImageData()
{
	string sImage_file = sConfig_path + "MH_05_cam0.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;
	
	// cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
		string imagePath = sData_path + "cam0/data/" + sImgFileName;

		Mat img = imread(imagePath.c_str(), 0);
		if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}
		pSystem->PubImageData(dStampNSec / 1e9, img);
		// cv::imshow("SOURCE IMAGE", img);
		// cv::waitKey(0);
		usleep(50000*nDelayTimes);
	}
	fsImage.close();
}

int main(int argc, char **argv)
{
	// 如果不是三段参数，就输出提示，表示输入错误
	if(argc != 3)
	{
		cerr << "./run_euroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config \n" 
			<< "For example: ./run_euroc /home/stevencui/dataset/EuRoC/MH-05/mav0/ ../config/"<< endl;
		return -1;
	}
	sData_path = argv[1]; // 数据集路径
	sConfig_path = argv[2]; // 配置文件路径

	pSystem.reset(new System(sConfig_path)); // 初始化系统
	
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem); // 后端主线程
		
	// sleep(5);
	std::thread thd_PubImuData(PubImuData);  // IMU数据的获取线程

	std::thread thd_PubImageData(PubImageData); // 图像数据的获取线程
	
	std::thread thd_Draw(&System::Draw, pSystem); // 单独的绘图线程已实现可视化
	
	thd_PubImuData.join();
	thd_PubImageData.join();

	// thd_BackEnd.join();
	// thd_Draw.join();

	cout << "main end... see you ..." << endl;
	return 0;
}
