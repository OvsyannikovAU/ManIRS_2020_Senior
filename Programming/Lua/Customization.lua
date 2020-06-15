--User functions------------------------------------------
function init()

end

function simulationStepStarted()
    --[[Example of using standart functions of robot:
    
    simTime=sim.getSimulationTime() -- get time of simulation
    --simTime - float number in seconds, e.g. simPime=0.563 [s]
        
    --Robot have gripper to take and realise objects:
    openGripper() --open gripper, time to full open ~1s
    closeGripper() -- close gripper, time to full close ~1s
    
    --Robot have a camera (resolution 256 x 256 pixeles):
    turnCameraToFront() --time to full move ~0.2s
    turnCameraToDown() --time to full move ~0.2s
    
    img=getCameraImage() --get full camera image in RGB mode, resolution 256 x 256 pixeles
    --Return number array with lenght=256*256*3=196608 values. Array struct:
    --{pix_0x0_red, pix_0x0_green, pix_0x0_blue, pix_0x1_red, pix_0x1_green, pix_0x1_blue...}
    -- Each value in range 0..1, e.g.
    --{0.80000001192093, 0.7843137383461, 0.76470589637756...}
    
    x=0
    y=0
    pixel=getCameraPixel(x,y) --get RGB pixel values from camera
    --Return number array with lenght 3. Array struct:
    --{pixel_red, pixel_green, pixel_blue}
    -- Each value in range 0..1, e.g.
    --{0.80000001192093, 0.7843137383461, 0.76470589637756}
    --Pixel numering in range 0..255 (camera resolution 256 x 256)
    
    x=0
    x_size=10
    y=0
    y_size=10
    imgField=getCameraField(x, x_size,y, y_size) --get  part of camera image in RGB mode
    --Must be x+x_size<255 and y+y_size<255
    --Pixel numering in range 0..255 (camera resolution 256 x 256)
    --Return number array with lenght x_size*y_size*3. Array struct:
    --{pix_X_Y_red, pix_X_Y_green, pix_X_Y_blue, pix_X_Y+1_red, pix_X_Y+1_green, pix_X_Y+1_blue...}
    -- Each value in range 0..1, e.g.
    --{0.80000001192093, 0.7843137383461, 0.76470589637756...}
    
    
    --Move robot link:
    pos=90 -- deg
    setPos1(pos) --set joint 1 to position
    --Pos - angle in degrees, range -180..180
    
    pos2=120 -- deg
    setPos2(pos) --set joint 2 to position
    --Pos - angle in degrees, range -150..150
    
    pos=100 -- mm
    setPos3(pos) --set joint 3 to position
    --Pos - interval in mm, range 0..160
    
    pos=-180 -- deg
    setPos4(pos) --set joint 4 to position
    --Pos - angle in degrees, range -180..180
    
    pos1=90 --deg
    pos2=120 --deg
    pos3=100 --mm
    pos4=-180 --deg
    setPositions(pos1,pos2,pos3,pos4) --set all joints to positions.
    --posX - same as parameters in previos functions
    
    kp=0.1
    ki=0.01
    kd=0.1
    link=link1
    setPID(link, kp, ki, kd) --set new PID-parameters to link
    -- kp, ki, kd - float num
        
    resetPID(link) --set to link default parameters kp=0.1, ki=0, kd=0
    
    link=link1
    speed=180 --deg/s or mm/s
    setMaxSpeed(link, speed) --set maximum speed to link
    
    --See also:
    --ReadMe for information about robot and simulator
    --https://coppeliarobotics.com/helpFiles/en/apiFunctionListCategory.htm - list of all Lua regular API function
    ]]--
end

function simulationStepDone()

end

function cleanup()

end
--Put your functions here







--system functions------------------------------------------
function sysCall_init()
    require "Programming/Lua/RobotFunctions"
    roboInit()
    init()    
end

function sysCall_actuation()
    simulationStepStarted()
end


function sysCall_sensing()
    link1_enc=sim.getJointPosition(link1)*180/math.pi
    link2_enc=sim.getJointPosition(link2)*180/math.pi
    link3_enc=sim.getJointPosition(link3)*1000
    link4_enc=sim.getJointPosition(link4)*180/math.pi
    cam_enc=sim.getJointPosition(linkSrv)*180/math.pi
    grip_enc=sim.getIntegerSignal('Gripper_pos')
    simulationStepDone()
end

function sysCall_cleanup()
    cleanup()
end

function sysCall_beforeSimulation()
    -- Called just before simulation starts. See also sysCall_afterSimulation
end

function remoteResetDyn(links)
    local ret=true
    for i=1,#links,1 do
        local res=sim.resetDynamicObject(links[i])
        ret=ret and res
    end
	return ret
end