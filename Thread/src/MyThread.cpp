//
// Created by luojunhui on 1/28/20.
//

#include <csignal>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <chrono>
#include <iostream>
#include <memory>
#include "MyThread.hpp"

using namespace std;
using namespace cv;

#if SAVE_LOG == 1
    std::ofstream logWrite("../Log/log.txt",ios::out);
#endif

#if SAVE_VIDEO == 1
extern int video_save_count;
VideoWriter videowriter;
#endif


namespace rm
{
    int8_t curControlState = AUTO_SHOOT_STATE; //current control mode

    bool ImgProdCons::quitFlag = false;

    bool pauseFlag = false;

#if SAVE_LOG == 1
    double timeFlag, taskTime;
#endif

    void ImgProdCons::SignalHandler(int)
    {
        LOGE("Process Shut Down By SIGINT\n");
        ImgProdCons::quitFlag = true;

#if SAVE_VIDEO == 1
        videowriter.release();
#endif

#if SAVE_LOG == 1
        logWrite.close();
#endif

        exit(-1);
    }


    void ImgProdCons::InitSignals()
    {
        ImgProdCons::quitFlag = false;

        struct sigaction sigact{};
        sigact.sa_handler = SignalHandler;
        sigemptyset(&sigact.sa_mask);
        sigact.sa_flags = 0;
        /*if interrupt occurs,set _quit_flag as true*/
        sigaction(SIGINT, &sigact, (struct sigaction *)nullptr);
    }


    ImgProdCons::ImgProdCons():
            serialPtr(unique_ptr<Serial>(new Serial())),
            solverPtr(unique_ptr<SolveAngle>(new SolveAngle())),
            armorDetectorPtr(unique_ptr<ArmorDetector>(new ArmorDetector())),
            energyPtr(unique_ptr<EnergyDetector>(new EnergyDetector())),
            driver(),
            missCount(0)
    {
    }

    void ImgProdCons::Init()
    {
        /*initialize signal*/
        InitSignals();

        /*initialize camera*/
        driver = &videoCapture;

        Mat curImage;
        if((driver->InitCam() && driver->SetCam() && driver->StartGrab()))
        {
            LOGM("Camera Initialized\n");
            LOGM("Camera Set Down\n");
            LOGM("Camera Start to Grab Frames\n");
        }
        else
        {
            driver->StopGrab();
            LOGW("Camera Resource Released\n");
            exit(-1);
        }
        do
        {
            if(driver->Grab(curImage))
            {
                FRAMEWIDTH = curImage.cols;
                FRAMEHEIGHT = curImage.rows;
                break;
            }
	    
            missCount++;
            if(missCount > 5)
            {
                driver->StopGrab();
                exit(-1);
            }
        }while(true);
        missCount = 0;

#if SAVE_VIDEO == 1
        string video_save_path = "../Log/video_" + std::to_string(video_save_count) + ".avi";
        videowriter = VideoWriter(video_save_path,cv::VideoWriter::fourcc('X', 'V', 'I', 'D'),60,Size(FRAMEWIDTH,FRAMEHEIGHT));
#endif
        LOGM("Initialization Completed\n");
    }

    void ImgProdCons::Produce()
    {
#if SAVE_LOG == 1
        timeFlag = (double)getTickCount();
#endif

        while (!driver->Grab(frame) || frame.rows != FRAMEHEIGHT || frame.cols != FRAMEWIDTH)
        {
            missCount++;
            LOGW("FRAME GRAB FAILED!\n");
            if(missCount > 5)
            {
                driver->StopGrab();
                raise(SIGINT);
            }
        }
	energy_frame = frame.clone();

        missCount = 0;

#if DEBUG_MSG == 1
            LOGM("Produce Thread Completed\n");
#endif

#if SAVE_VIDEO == 1
            videowriter.write(frame);
#endif

#if SAVE_LOG == 1
            logWrite<<"[Produce Time Consume] : "<<((double)getTickCount() - timeFlag)/getTickFrequency()<<endl;
#endif

    }

    void ImgProdCons::Energy()
    {
#if SAVE_LOG == 1
        taskTime = (double)getTickCount();
#endif
        /*do energy detection*/
        energyPtr->EnergyTask(frame, true, (double)getTickCount());

#if SAVE_LOG == 1
        logWrite<<"[Energy Time Consume] : "<<((double)getTickCount() - taskTime)/getTickFrequency()<<endl;
#endif

    }

    void ImgProdCons::Feedback()
    {

#if SAVE_LOG == 1
            taskTime = (double)getTickCount();
#endif
            /*do energy things*/
            if(showEnergy)
            {
//                circle(frame,energyPtr->predict_point,5,Scalar(128,128,0),-1);
//                circle(frame,energyPtr->circle_center_point,5,Scalar(0,128,128),-1);
//                
//                for(auto pts:energyPtr->target_armor_centers)
//                {
//                    circle(frame,pts,5,Scalar(128,0,0),-1);
//                }
                imshow("energy",energy_frame);
            }

           // solverPtr->GetPoseV(Point2f(0, 0),
           //                     energyPtr->pts,
           //                     15, false);

            serialPtr->pack(receiveData.yawAngle + solverPtr->yaw,receiveData.pitchAngle + solverPtr->pitch, solverPtr->dist, solverPtr->shoot,
                            true, BIG_ENERGY_STATE,0);

	    	LOGM("Pack Complete");
            /**press key 'p' to pause or continue task**/
            if(showOrigin || showEnergy)
            {
                if(!pauseFlag && waitKey(30) == 'p'){pauseFlag = true;}
				//pauseFlag = false;
				//For usage of debugging, press 'p' to continue the next frame.                
				if(pauseFlag)
                {
                    while(waitKey() != 'p'){}
                    pauseFlag = false;
                }
				
            }
		
            /** send data from host to low-end machine to instruct holder's movement **/
            if(serialPtr->WriteData())
            {
#if SAVE_LOG == 1
                logWrite<<"[Write Data to USB2TTL SUCCEED]"<<endl;
#endif
            }
            else
            {
                logWrite<<"[Write Data to USB2TTL FAILED]"<<endl;
            }

            /**Receive data from low-end machine to update parameters(the color of robot, the task mode, etc)**/
            if(serialPtr->ReadData(receiveData))
            {
#if DEBUG_MSG == 1
                LOGM("Receive Data\n");
#endif
                /**Update task mode, if receiving data failed, the most reasonable decision may be just keep the status
                 * as the last time**/
//                curControlState = receiveData.targetMode;

                /**because the logic in armor detection task need the color of enemy, so we need to negate to color variable
                 * received, receiveData.targetColor means the color of OUR robot, but not the enemy's**/
                blueTarget = (receiveData.targetColor) == 0;
		
#if SAVE_LOG == 1
                logWrite<<"[Feedback Time Consume] : "<<((double)getTickCount() - taskTime)/getTickFrequency()<<endl;
                logWrite<<"[Total Time Consume] : "<<((double)getTickCount() - timeFlag)/getTickFrequency()<<endl;
                logWrite<<"[Current Yaw Angle] : "<<receiveData.yawAngle<<endl;
                logWrite<<"[Current Pitch Angle] : "<<receiveData.pitchAngle<<endl;
#endif

#if DEBUG_MSG == 1
                LOGM("BlueTarget: %d\n",(int)blueTarget);
#endif
            }
            else
            {
#if SAVE_LOG == 1
                logWrite<<"[Receive Data from USB2TTL FAILED]"<<endl;
#endif
            }

#if SAVE_LOG == 1
            logWrite<<"============================Process Finished============================"<<endl;
#endif

#if DEBUG_MSG == 1
            LOGM("Feedback Thread Completed\n");
#endif
    }
}

