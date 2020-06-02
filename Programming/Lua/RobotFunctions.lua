function roboInit()
	link1=sim.getObjectHandle('Link1')
    link2=sim.getObjectHandle('Link2')
    link3=sim.getObjectHandle('Link3')
    link4=sim.getObjectHandle('Link4')
    linkSrv=sim.getObjectHandle('CameraServo')
    cam=sim.getObjectHandle('RoboCamera')
    link1_enc=sim.getJointPosition(link1)*180/math.pi
    link2_enc=sim.getJointPosition(link2)*180/math.pi
    link3_enc=sim.getJointPosition(link3)*1000
    link4_enc=sim.getJointPosition(link4)*180/math.pi
	cam_enc=sim.getJointPosition(linkSrv)*180/math.pi
	grip_enc=sim.getIntegerSignal('Gripper_pos')
	if grip_enc==nil then grip_enc=0 end
end

function setPos1(pos)
    local p=pos
    if p>180 then p=180 end
    if p<-180 then p=-180 end
    sim.setJointTargetPosition(link1, p*math.pi/180)
end

function setPos2(pos)
local p=pos
    if p>150 then p=150 end
    if p<-150 then p=-150 end
    sim.setJointTargetPosition(link2, p*math.pi/180)
end

function setPos3(pos)
    local p=pos
    if p>160 then p=160 end
    if p<0 then p=0 end
    sim.setJointTargetPosition(link3, p*0.001)
end

function setPos4(pos)
    local p=pos
    if p>180 then p=180 end
    if p<-180 then p=-180 end
    sim.setJointTargetPosition(link4, p*math.pi/180)
end

function setPositions(pos1, pos2, pos3, pos4)
    setPos1(pos1)
    setPos2(pos2)
    setPos3(pos3)
    setPos4(pos4)
end

function closeGripper()
    sim.setIntegerSignal('Gripper_close', 1)
end

function openGripper()
    sim.setIntegerSignal('Gripper_close', 0)
end

function turnCameraToFront()
    sim.setJointTargetPosition(linkSrv, 90*math.pi/180)
end

function turnCameraToDown()
    sim.setJointTargetPosition(linkSrv, 0*math.pi/180)
end

function getCameraImage()
    local img=sim.getVisionSensorImage(cam, 0,0,0,0,0)
    return img
end

function getCameraPixel(x, y)
    local img=sim.getVisionSensorImage(cam, x,y,1,1,0)
    return img
end

function getCameraField(x, x_size, y, y_size)
    local img=sim.getVisionSensorImage(cam, x,y,x_size,y_size,0)
    return img
end

function setCameraResolution(x, y)
	local resX=x
	local resY=y
	if resX<1 then resX=1 end
	if resX>1024 then resY=1024 end
	if resY<1 then resY=1 end
	if resY>1024 then resY=1024 end
	local resultX = sim.setObjectInt32Parameter(cam,sim.visionintparam_resolution_x, resX)
	local resultY = sim.setObjectInt32Parameter(cam,sim.visionintparam_resolution_y, resY)
	return resultX and resultY
end

function setPID(link,kp,ki,kd)
    sim.setObjectFloatParameter(link, sim.jointfloatparam_pid_p ,kp)
    sim.setObjectFloatParameter(link, sim.jointfloatparam_pid_i ,ki) 
    sim.setObjectFloatParameter(link, sim.jointfloatparam_pid_d ,kd) 
end

function resetPID(link)
    setPID(link, 0.1,0,0)
end

function setMaxSpeed(link, speed)
	local spd=speed
	JT=sim.getJointType(link)
    if JT>-1 then
        if JT==sim.joint_revolute_subtype then spd=speed*math.pi/180 end
        if JT==sim.joint_prismatic_subtype then spd=speed*0.001 end
    end
    sim.setObjectFloatParameter(link, sim.jointfloatparam_upper_limit, spd)
	sim.resetDynamicObject(link)
end