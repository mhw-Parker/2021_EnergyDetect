#include <thread>
#include <fstream>
#include "MyThread.hpp"
#include "preoptions.h"

using namespace rm;
using namespace cv;
using namespace std;

#if SAVE_VIDEO == 1
int video_save_count;
#endif

int main(int argc, char** argv)
{
    PreOptions(argc,argv);

#if SAVE_VIDEO == 1
    std::ifstream fileVideoCountRead("../Log/video_count_file.txt", ios::in);
    if(!fileVideoCountRead.is_open())
    {
        LOGE("VIDEO SAVE FAILED\n");
        video_save_count = 0;
    }
    else
    {
        fileVideoCountRead >> video_save_count;
    }
    fileVideoCountRead.close();

    std::ofstream fileVideoCountWrite("../Log/video_count_file.txt", ios::out);
    if(!fileVideoCountWrite.is_open())
        LOGE("VIDEO SAVE FAILED\n");
    fileVideoCountWrite << video_save_count + 1 << endl;
    fileVideoCountWrite.close();
#endif

    ImgProdCons pro;
    pro.Init();

    do {
	for(int i = 0; i < 5; i++)
        	pro.Produce();
        pro.Energy();
        pro.Feedback();
    } while (true);

    return 0;
}
