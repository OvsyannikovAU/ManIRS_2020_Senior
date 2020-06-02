import os
os.add_dll_directory(r'C:\Program Files\CoppeliaRobotics\CoppeliaSimEdu')

import b0RemoteApi
from RoboFunctions import ManRobot
import time


with b0RemoteApi.RemoteApiClient('b0RemoteApi_pythonClient','b0RemoteApi_manirs') as client:
    doNextStep=True
    robot=ManRobot(client)
    step=0 #переменная для подсчета шагов симуляции

    def init():
	#пример использования функций управления роботом:
        #подробнее о всех функицях, их параметрах и возвращаемых значения см ReadMe

        #энкодеры:
        #robot.link1_enc
        #robot.link2_enc
        #robot.link3_enc
        #robot.link4_enc
        #robot.grip_enc
        #robot.cam_enc
        
        #simTime=robot.simTime
        
        #pos1=90 #градусов
        #pos2=90 #градусов
        #pos3=100 #мм
        #pos4=90 #градусов
        #robot.setPos1(pos1)
        #robot.setPos2(pos2)
        #robot.setPos3(pos3)
        #robot.setPos4(pos4)
        #robot.setPositions(pos1, pos2, pos3, pos4)

        #kp=0.1
        #ki=0.01
        #kd=0.1
        #robot.setPID(robot.link1,kp,ki,kd)
        #robot.resetPID(robot.link1)

        #speed=180   #максимальная скорость звена в мм/с или град/с
        #robot.setMaxSpeed(robot.link1, speed) #установить максимальную скорость для выбранного звена

        #robot.closeGripper()
        #robot.openGripper()
        #robot.turnCameraToFront()
        #robot.turnCameraToDown()

        #resX=256 #Высота кадра в пикселях, диапазон от 1 до 1024 пикселей
        #resY=256 #Ширина кадра в пикселях, диапазон от 1 до 1024 пикселей
        #robot.setCameraResolution(resX, resY) #установить разрешение камеры робота

        #img=robot.cam_image
        time.time()
        
	#See also:
	#ReadMe for information about robot and simulator
	#https://coppeliarobotics.com/helpFiles/en/b0RemoteApi-python.htm - list of all Python B0 remote API function
	
    def simulationStepStarted(msg):
        #бездействие
        time.time()
        
    def simulationStepDone(msg):
        #бездействие
        global step #выводим текущий шаг
        print(step) #просто для демонстрации, можно отключить
        global doNextStep #а это обязательный участок кода
        doNextStep=True   #для синхронизации с основным потоком ниже
	
    def cleanup():
	#бездействие
        time.time()
		
    client.simxSynchronous(True)
    client.simxGetSimulationStepStarted(client.simxDefaultSubscriber(simulationStepStarted))
    client.simxGetSimulationStepDone(client.simxDefaultSubscriber(simulationStepDone))	
    res=client.simxStartSimulation(client.simxDefaultPublisher())
    init()
	#Put your main action here:
	
    startTime=time.time()
    startStep=step
    while robot.simTime<5: #крутить цикл 5 секунд с начала симуляции
        #варианты условий для выхода из цикла:
        #time.time()<startTime+5 #крутить 5 секунд саму программу
        #step-startStep<100 #крутить цикл 100 шагов симуляции
        if doNextStep:
            doNextStep=False
            #действия в цикле, синхронизированном с симулятором

            step=step+1
            client.simxSynchronousTrigger()
        client.simxSpinOnce()
	
    #тут могут быть несинхронизированные с симулятором действия,
    #блокирующие функции:
    #time.sleep(надолго)
	
    #End of simulation:
    cleanup()
    client.simxStopSimulation(client.simxDefaultPublisher())
	
