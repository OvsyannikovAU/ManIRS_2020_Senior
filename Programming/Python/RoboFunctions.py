from math import pi

class ManRobot:
	link1=0
	link2=0
	link3=0
	link4=0
	linkSrv=0
	cam=0	
	link1_enc=0.0
	link2_enc=0.0
	link3_enc=0.0
	link4_enc=0.0
	grip_enc=0.0
	cam_enc=0.0
	client=0
	simTime=0
	cam_image=[]
	__simState=0
	
	def __init__(self, cl):
		self.client=cl
		errHand, self.link1=self.client.simxGetObjectHandle('Link1', self.client.simxServiceCall() )
		errHand, self.link2=self.client.simxGetObjectHandle('Link2', self.client.simxServiceCall() )
		errHand, self.link3=self.client.simxGetObjectHandle('Link3', self.client.simxServiceCall() )
		errHand, self.link4=self.client.simxGetObjectHandle('Link4', self.client.simxServiceCall() )
		errHand, self.linkSrv=self.client.simxGetObjectHandle('CameraServo', self.client.simxServiceCall() )
		errHand, self.cam=self.client.simxGetObjectHandle('RoboCamera', self.client.simxServiceCall() )
		
		self.client.simxGetJointPosition(self.link1, self.client.simxDefaultSubscriber(self.f_link1_enc) )
		self.client.simxGetJointPosition(self.link2, self.client.simxDefaultSubscriber(self.f_link2_enc) )
		self.client.simxGetJointPosition(self.link3, self.client.simxDefaultSubscriber(self.f_link3_enc) )
		self.client.simxGetJointPosition(self.link4, self.client.simxDefaultSubscriber(self.f_link4_enc) )
		self.client.simxGetJointPosition(self.linkSrv, self.client.simxDefaultSubscriber(self.f_cam_enc) )
		self.client.simxGetIntSignal('Gripper_pos', self.client.simxDefaultSubscriber(self.f_grip_enc) )
		self.client.simxGetSimulationTime( self.client.simxDefaultSubscriber(self.getSimTime) )
		self.client.simxGetSimulationState( self.client.simxDefaultSubscriber(self.getSimState) )
		self.client.simxGetVisionSensorImage(self.cam, False, self.client.simxDefaultSubscriber(self.getCameraImage,1) )
		
	def setPos1(self, pos=0):
		p=pos
		if p>180:
			p=180
		if p<-180:
			p=-180
		errPos=self.client.simxSetJointTargetPosition(self.link1, p*pi/180, self.client.simxDefaultPublisher() )
		return errPos

	def setPos2(self, pos=0):
		p=pos
		if p>150:
			p=150
		if p<-150:
			p=-150
		errPos=self.client.simxSetJointTargetPosition(self.link2, p*pi/180, self.client.simxDefaultPublisher() )
		return errPos
	
	def setPos3(self, pos=0):
		p=pos
		if p>160:
			p=160
		if p<0:
			p=0
		errPos=self.client.simxSetJointTargetPosition(self.link3, p*0.001, self.client.simxDefaultPublisher() )
		return errPos
	
	def setPos4(self, pos=0):
		p=pos
		if p>180:
			p=180
		if p<-180:
			p=-180
		errPos=self.client.simxSetJointTargetPosition(self.link4, p*pi/180, self.client.simxDefaultPublisher() )
		return errPos
		
	def setPositions(self, pos1=0, pos2=0, pos3=0, pos4=0):
		self.setPos1(pos1)
		self.setPos2(pos2)
		self.setPos3(pos3)
		self.setPos4(pos4)
		#return self.setPos1(pos1) and self.setPos2(pos2) and self.setPos3(pos3) and self.setPos4(pos4)
	
	def closeGripper(self):
		errSig=self.client.simxSetIntSignal('Gripper_close', 1, self.client.simxDefaultPublisher() )
		return errSig
	
	def openGripper(self):
		errSig=self.client.simxSetIntSignal('Gripper_close', 0, self.client.simxDefaultPublisher() )
		return errSig
	
	def turnCameraToFront(self):
		errPos=self.client.simxSetJointTargetPosition(self.linkSrv, 90*pi/180, self.client.simxDefaultPublisher() )
		return errPos
	
	def turnCameraToDown(self):
		errPos=self.client.simxSetJointTargetPosition(self.linkSrv, 0*pi/180, self.client.simxDefaultPublisher() )
		return errPos
	
	def getCameraImage(self, msg):
		if msg[0] and self.__simState>0:
			self.cam_image=msg[2]
			
	def setCameraResolution(self, x, y):
		resX=x
		resY=y
		if resX<1: resX=1
		if resX>1024: resX=1024
		if resY<1: resY=1
		if resY>1024: resY=1024
		errParX=self.client.simxSetObjectIntParameter(self.cam, 1002, resX, self.client.simxDefaultPublisher() )
		errParY=self.client.simxSetObjectIntParameter(self.cam, 1003, resY, self.client.simxDefaultPublisher() )
	
	def setPID(self,link,kp,ki,kd):
		self.client.simxSetObjectFloatParameter(link, 2002, kp, self.client.simxDefaultPublisher() )
		self.client.simxSetObjectFloatParameter(link, 2003, ki, self.client.simxDefaultPublisher() )
		self.client.simxSetObjectFloatParameter(link, 2004, kd, self.client.simxDefaultPublisher() )
	
	def resetPID(self,link):
		self.setPID(link, 0.1,0,0)
	
	def setMaxSpeed(self,link, speed):
		args=[link, speed]
		ret=self.client.simxCallScriptFunction('remoteResetDyn@ManIRS_senior_robot','sim.scripttype_customizationscript',args,self.client.simxServiceCall())
		return ret
		
	def getSimTime(self, msg):
		if msg[0] and self.__simState>0:
			self.simTime=msg[1]
		
	def f_link1_enc(self,msg):
		if msg[0] and self.__simState>0:
			self.link1_enc=msg[1]*180.0/pi
	
	def f_link2_enc(self,msg):
		if msg[0] and self.__simState>0:
			self.link2_enc=msg[1]*180.0/pi
	
	def f_link3_enc(self,msg):
		if msg[0] and self.__simState>0:
			self.link3_enc=msg[1]*1000
	
	def f_link4_enc(self,msg):
		if msg[0] and self.__simState>0:
			self.link4_enc=msg[1]*180.0/pi
	
	def f_cam_enc(self,msg):
		if msg[0] and self.__simState>0:
			self.cam_enc=msg[1]*180.0/pi
	
	def f_grip_enc(self,msg):
		if msg[0] and self.__simState>0:
			self.grip_enc=msg[1]
		else:
			self.grip_enc=0
			
	def getSimState(self, msg):
		if msg[0]:
			self.__simState=msg[1]