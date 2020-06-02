#include <iostream>
#include "b0RemoteApi.h"
#pragma once
class ManRobot
{
public:
	int link1 = 0;
	int link2 = 0;
	int link3 = 0;
	int link4 = 0;
	int linkSrv = 0;
	int cam = 0;
	float link1_enc = 0.0;
	float link2_enc = 0.0;
	float link3_enc = 0.0;
	float link4_enc = 0.0;
	float grip_enc = 0.0;
	float cam_enc = 0.0;
	float simTime = 0.0;
	std::string cam_image="987";

	ManRobot(b0RemoteApi *cl);
	bool setPos1(float pos);
	bool setPos2(float pos);
	bool setPos3(float pos);
	bool setPos4(float pos);
	bool setPositions(float pos1, float pos2, float pos3, float pos4);
	bool closeGripper();
	bool openGripper();
	bool turnCameraToFront();
	bool turnCameraToDown();
	void setPID(int link, float kp, float ki, float kd);
	void resetPID(int link);
	void setMaxSpeed(int link, float speed);
	void getCameraImage(std::vector<msgpack::object>* msg);
	void setCameraResolution(int x, int y);
	void getSimTime(std::vector<msgpack::object>* msg);
	void getLink1Enc(std::vector<msgpack::object>* msg);
	void getLink2Enc(std::vector<msgpack::object>* msg);
	void getLink3Enc(std::vector<msgpack::object>* msg);
	void getLink4Enc(std::vector<msgpack::object>* msg);
	void getCamEnc(std::vector<msgpack::object>* msg);
	void getGripEnc(std::vector<msgpack::object>* msg);
	void getSimState(std::vector<msgpack::object>* msg);


private:
	b0RemoteApi *client = NULL;
	int simState = 0;
};

