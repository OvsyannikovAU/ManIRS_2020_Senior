// ManIRS_senior.cpp : Этот файл содержит функцию "main". Здесь начинается и заканчивается выполнение программы.
//

#include <iostream>
#include <string>
#include "b0RemoteApi.h"
#include "ManRobot.h"


bool doNextStep = true;
bool runInSynchronousMode = true;
int step = 0;
b0RemoteApi client("b0RemoteApi_c++Client", "b0RemoteApi_manirs");
ManRobot robot(&client);

void init()
{
    //пример использования функций управления роботом:
    //подробнее о всех функицях, их парметрах и возвращаемых значения см ReadMe

    //энкодеры:
    //float enc1=robot.link1_enc;
    //float enc2=robot.link2_enc;
    //float enc3=robot.link3_enc;
    //float enc4=robot.link4_enc;
    //int encGrip=robot.grip_enc;
    //float encCam=robot.cam_enc;

    //float simTime = robot.simTime;

    //float pos1 = 90;  //градусов
    //float pos2 = 90;  //градусов
    //float pos3 = 100; //миллиметров
    //float pos4 = 90;  //градусов
    //robot.setPos1(pos1);
    //robot.setPos2(pos2);
    //robot.setPos3(pos3);
    //robot.setPos4(pos4);
    //robot.setPositions(pos1, pos2, pos3, pos4);

    //float kp = 0.1;
    //float ki = 0.01;
    //float kd = 0.1;
    //robot.setPID(robot.link1, kp, ki, kd);
    //robot.resetPID(robot.link1);

    //float speed = 180; //max speed in mm/s or deg/s
    //robot.setMaxSpeed(robot.link1, speed);

    //robot.closeGripper();
    //robot.openGripper();
    //robot.turnCameraToDown();
    //robot.turnCameraToFront();

    int resX = 2; //X resolution of RoboCamera in pixels 1...1024
    int resY = 2; //Y resolution of RoboCamera in pixels 1...1024
    robot.setCameraResolution(resX, resY);

    //std::string img = robot.cam_image; //кадр с камеры возвращается как строка длинной 196608 символов
    //Каждые три символа надо переводить в byte/int и обрабатывать как RGB-компоненты одного пикселя
	
	//См. также:
	//ReadMe для более подробной информации о роботе и симуляторе
	//https://coppeliarobotics.com/helpFiles/en/b0RemoteApi-cpp.htm - список всех C++ B0 remote API функций
}

void simulationStepStarted(std::vector<msgpack::object>* msg)
{

}

void simulationStepDone(std::vector<msgpack::object>* msg)
{
    //выводим текущий шаг, просто для демонстрации:
    std::cout << step << std::endl;
    doNextStep = true;
}

void cleanup()
{

}

void stepSimulation()
{
    if (runInSynchronousMode)
    {
        while (!doNextStep)
            client.simxSpinOnce();
        doNextStep = false;
        //действия в цикле, синхронизированном с симулятором
        
        step = step + 1;
        client.simxSynchronousTrigger();
    }
    else
        client.simxSpinOnce();
}

int main()
{
    if (runInSynchronousMode)
        client.simxSynchronous(true);

    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted));
    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone));
    client.simxStartSimulation(client.simxDefaultPublisher());
    init();
    //Put your main action here:

    int startStep = step;
    while (robot.simTime<5)
        stepSimulation();
	
	//тут могут быть несинхронизированные с симулятором действия,
    //и блокирующие функции
	
    //End of simulation:
    cleanup();
    client.simxStopSimulation(client.simxDefaultPublisher());
}

// Запуск программы: CTRL+F5 или меню "Отладка" > "Запуск без отладки"
// Отладка программы: F5 или меню "Отладка" > "Запустить отладку"
