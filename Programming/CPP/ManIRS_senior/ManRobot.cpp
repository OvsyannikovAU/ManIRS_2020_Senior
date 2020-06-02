#include "ManRobot.h"
#define PI 3.14159265358979323846

ManRobot::ManRobot(b0RemoteApi *cl)
{
	client = cl;
	std::vector<msgpack::object>* reply1 = client->simxGetObjectHandle("Link1", client->simxServiceCall());
	link1 = b0RemoteApi::readInt(reply1, 1);
	std::vector<msgpack::object>* reply2 = client->simxGetObjectHandle("Link2", client->simxServiceCall());
	link2 = b0RemoteApi::readInt(reply2, 1);
	std::vector<msgpack::object>* reply3 = client->simxGetObjectHandle("Link3", client->simxServiceCall());
	link3 = b0RemoteApi::readInt(reply3, 1);
	std::vector<msgpack::object>* reply4 = client->simxGetObjectHandle("Link4", client->simxServiceCall());
	link4 = b0RemoteApi::readInt(reply4, 1);
	std::vector<msgpack::object>* reply5 = client->simxGetObjectHandle("CameraServo", client->simxServiceCall());
	linkSrv = b0RemoteApi::readInt(reply5, 1);
	std::vector<msgpack::object>* reply6 = client->simxGetObjectHandle("RoboCamera", client->simxServiceCall());
	cam = b0RemoteApi::readInt(reply6, 1);

	boost::function<void(std::vector<msgpack::object>*)> fp1 = boost::bind(&ManRobot::getLink1Enc, this, _1);
	client->simxGetJointPosition(link1, client->simxDefaultSubscriber(fp1, 1));
	boost::function<void(std::vector<msgpack::object>*)> fp2 = boost::bind(&ManRobot::getLink2Enc, this, _1);
	client->simxGetJointPosition(link2, client->simxDefaultSubscriber(fp2, 1));
	boost::function<void(std::vector<msgpack::object>*)> fp3 = boost::bind(&ManRobot::getLink3Enc, this, _1);
	client->simxGetJointPosition(link3, client->simxDefaultSubscriber(fp3, 1));
	boost::function<void(std::vector<msgpack::object>*)> fp4 = boost::bind(&ManRobot::getLink4Enc, this, _1);
	client->simxGetJointPosition(link4, client->simxDefaultSubscriber(fp4, 1));
	boost::function<void(std::vector<msgpack::object>*)> fp5 = boost::bind(&ManRobot::getCamEnc, this, _1);
	client->simxGetJointPosition(linkSrv, client->simxDefaultSubscriber(fp5, 1));
	boost::function<void(std::vector<msgpack::object>*)> fp6 = boost::bind(&ManRobot::getGripEnc, this, _1);
	client->simxGetIntSignal("Gripper_pos", client->simxDefaultSubscriber(fp6, 1));
	boost::function<void(std::vector<msgpack::object>*)> fp7 = boost::bind(&ManRobot::getSimState, this, _1);
	client->simxGetSimulationState(client->simxDefaultSubscriber(fp7, 1));
	boost::function<void(std::vector<msgpack::object>*)> fp8 = boost::bind(&ManRobot::getSimTime, this, _1);
	client->simxGetSimulationTime(client->simxDefaultSubscriber(fp8, 1));
	boost::function<void(std::vector<msgpack::object>*)> fp9 = boost::bind(&ManRobot::getCameraImage, this, _1);
	client->simxGetVisionSensorImage(cam, false, client->simxDefaultSubscriber(fp9, 1));
}

bool ManRobot::closeGripper()
{
	bool errSig = client->simxSetIntSignal("Gripper_close", 1, client->simxDefaultPublisher());
	return errSig;
}

bool ManRobot::openGripper()
{
	bool errSig = client->simxSetIntSignal("Gripper_close", 0, client->simxDefaultPublisher());
	return errSig;
}

bool ManRobot::setPos1(float pos)
{
	float p = pos;
	if (p > 180) { p = 180; }
	if (p < -180) { p = -180; }
	bool errPos = client->simxSetJointTargetPosition(link1, p * PI / 180, client->simxDefaultPublisher());
	return errPos;
}

bool ManRobot::setPos2(float pos)
{
	float p = pos;
	if (p > 150) { p = 150; }
	if (p < -150) { p = -150; }
	bool errPos = client->simxSetJointTargetPosition(link2, p * PI / 180, client->simxDefaultPublisher());
	return errPos;
}

bool ManRobot::setPos3(float pos)
{
	float p = pos;
	if (p > 160) { p = 160; }
	if (p < 0) { p = 0; }
	bool errPos = client->simxSetJointTargetPosition(link3, p * 0.001, client->simxDefaultPublisher());
	return errPos;
}

bool ManRobot::setPos4(float pos)
{
	float p = pos;
	if (p > 180) { p = 180; }
	if (p < -180) { p = -180; }
	bool errPos = client->simxSetJointTargetPosition(link4, p * PI / 180, client->simxDefaultPublisher());
	return errPos;
}

bool ManRobot::setPositions(float pos1, float pos2, float pos3, float pos4)
{
	return setPos1(pos1) && setPos2(pos2) && setPos3(pos3) && setPos4(4);
}

void ManRobot::setPID(int link, float kp, float ki, float kd)
{
	client->simxSetObjectFloatParameter(link, 2002, kp, client->simxDefaultPublisher());
	client->simxSetObjectFloatParameter(link, 2003, ki, client->simxDefaultPublisher());
	client->simxSetObjectFloatParameter(link, 2004, kd, client->simxDefaultPublisher());
}

void ManRobot::resetPID(int link)
{
	setPID(link, 0.1, 0, 0);
}

void ManRobot::setMaxSpeed(int link, float speed)
{
	std::tuple <int, float > args(link, speed);
	std::stringstream packedArgs;
	msgpack::pack(packedArgs, args);
	std::vector<msgpack::object>* reply = client->simxCallScriptFunction("remoteResetDyn@ManIRS_senior_robot", "sim.scripttype_customizationscript", packedArgs.str().c_str(), packedArgs.str().size(), client->simxServiceCall());
}

bool ManRobot::turnCameraToDown()
{
	bool errPos = client->simxSetJointTargetPosition(linkSrv, 0 * PI / 180, client->simxDefaultPublisher());
	return errPos;
}

bool ManRobot::turnCameraToFront()
{
	bool errPos = client->simxSetJointTargetPosition(linkSrv, 90 * PI / 180, client->simxDefaultPublisher());
	return errPos;
}

void ManRobot::getCameraImage(std::vector<msgpack::object>* msg)
{
	bool errRead = b0RemoteApi::readInt(msg, 0);
	if (!errRead && simState > 0) 
	{
		cam_image=b0RemoteApi::readByteArray(msg, 2);
	} 
}

void ManRobot::setCameraResolution(int x, int y)
{
	int resX = x;
	int resY = y;
	if (resX < 1) { resX = 1; }
	if (resX > 1024) { resX = 1024; }
	if (resY < 1) { resY = 1; }
	if (resY > 1024) { resY = 1024; }
	client->simxSetObjectIntParameter(cam, 1002, resX, client->simxDefaultPublisher());
	client->simxSetObjectIntParameter(cam, 1003, resY, client->simxDefaultPublisher());
}

void ManRobot::getLink1Enc(std::vector<msgpack::object>* msg)
{
	bool errRead = b0RemoteApi::readInt(msg, 0);
	if (!errRead && simState>0) { link1_enc = b0RemoteApi::readFloat(msg, 1)*180.0/PI; }
}

void ManRobot::getLink2Enc(std::vector<msgpack::object>* msg)
{
	bool errRead = b0RemoteApi::readInt(msg, 0);
	if (!errRead && simState > 0) { link2_enc = b0RemoteApi::readFloat(msg, 1) * 180.0 / PI; }
}

void ManRobot::getLink3Enc(std::vector<msgpack::object>* msg)
{
	bool errRead = b0RemoteApi::readInt(msg, 0);
	if (!errRead && simState > 0) { link3_enc = b0RemoteApi::readFloat(msg, 1) * 1000; }
}

void ManRobot::getLink4Enc(std::vector<msgpack::object>* msg)
{
	bool errRead = b0RemoteApi::readInt(msg, 0);
	if (!errRead && simState > 0) { link4_enc = b0RemoteApi::readFloat(msg, 1) * 180.0 / PI; }
}

void ManRobot::getCamEnc(std::vector<msgpack::object>* msg)
{
	bool errRead = b0RemoteApi::readInt(msg, 0);
	if (!errRead && simState > 0) { cam_enc = b0RemoteApi::readFloat(msg, 1) * 180.0 / PI; }
}

void ManRobot::getGripEnc(std::vector<msgpack::object>* msg)
{
	bool errRead = b0RemoteApi::readInt(msg, 0);
	if (!errRead && simState > 0) { grip_enc = b0RemoteApi::readInt(msg, 1); }
	else { grip_enc = 0; }
}

void ManRobot::getSimState(std::vector<msgpack::object>* msg)
{
	bool errRead= b0RemoteApi::readInt(msg, 0);
	if (!errRead) { simState = b0RemoteApi::readInt(msg, 1); }
}

void ManRobot::getSimTime(std::vector<msgpack::object>* msg)
{
	bool errRead = b0RemoteApi::readInt(msg, 0);
	if (!errRead) { simTime = b0RemoteApi::readFloat(msg, 1); }
}