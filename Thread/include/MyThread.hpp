//
// Created by root on 2021/1/14.
//
#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include <mutex>
#include <memory>
#include <shared_mutex>
#include <condition_variable>
#include <sys/timeb.h>
#include <sys/syscall.h>

#include <Media/RMDriver.h>

#include "ArmorDetector.hpp"
#include "SerialPort.hpp"
#include "SolveAngle.hpp"
#include "preoptions.h"
#include "mydefine.h"
#include "Filter.h"
#include "EnergyDetector.h"
#include "VideoDriver.hpp"

#include "utility.hpp"

using namespace RMTools;
using namespace std;

namespace rm
{

    class ImgProdCons
    {
    public:
        ImgProdCons();

        ~ImgProdCons() {};

        /*initialize*/
        void Init();

        /*
        * @Brief: Receive self state from the serial port, update task mode if commanded
        */
        void Feedback();

        /*
        * @Brief: keep reading image from the camera into the buffer
        */
        void Produce();

        /*
         * @Brief: energy
         */
        void Energy();

    public:
        /* Camera */
        Driver *driver;
    private:

        static void SignalHandler(int);
        static void InitSignals(void);

        /*Camera Driver Instances*/
        RMDriver dahuaCapture;

        VideoDriver videoCapture;

        /* Serial */
        std::unique_ptr<Serial> serialPtr;

        /* Angle solver */
        std::unique_ptr<SolveAngle> solverPtr;

        /* Armor detector */
        std::unique_ptr<ArmorDetector> armorDetectorPtr;

        /*EnergyDetector buffer detector*/
        std::unique_ptr<EnergyDetector> energyPtr;

        Mat frame;

        struct ReceiveData receiveData;

        int missCount;

	Mat energy_frame;
        static bool quitFlag;
    };

}

