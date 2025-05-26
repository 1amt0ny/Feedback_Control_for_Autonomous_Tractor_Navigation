function sysCall_init()
    sim = require('sim')
    
    file = io.open('PI.txt', 'w+')
    file:write('Velocity and Turning Rate of the final trajectory:\n\n')
    
    accuracy = io.open('trajectory.txt', 'w')
    accuracy:write("Time, X, Y\n")
    
    -- Global variables for poses initialized
    targetPose = {x, y, phi}
    currentPose = {x, y, heading} 
    distance = 0
    turnRateControl = 1
    velocityControl = 1

    -- Global variables for control
    -- integrator for turning rate 
    intRot=0.0;
    -- integrator for velocity
    intVel=0.0;
    -- two control variables
    uOmega=0.0; -- for turning rate
    uV=0.0; -- for velocity

    wRef = 0.0
    uRef = 0.0

    -- Global variables for sensing initialized
    phi = 0
    theta = 0
    alpha = 0
    
    uMes = 0.0
    wMes = 0.0

    -- control parameters: 
    k = 3.0/50
    gamma = 3.0/10
    h = 3.0/1
    
    --k = 3.0/50
    --gamma = 3.0/30
    --h = 3.0/0.5
    
    

    -- motor handles
    motorRight1=sim.getObjectHandle("Motor4")
    motorRight2=sim.getObjectHandle("Motor3")
    motorLeft1=sim.getObjectHandle("Motor1")
    motorLeft2=sim.getObjectHandle("Motor2")
    
    handleGraph = sim.getObject('/Graph')
    stream2 = sim.addGraphStream(handleGraph, 'turning_rate', '', 0, {1, 1, 0}, 0)
    stream1 = sim.addGraphStream(handleGraph, 'ref_turning_rate', '', 0, {1, 0, 0}, 0)
    
    handleGraph1 = sim.getObject('/Graph1')
    stream4 = sim.addGraphStream(handleGraph1, 'velocity', '', 0, {1, 1, 0}, 0)
    stream3 = sim.addGraphStream(handleGraph1, 'ref_velocity', '', 0, {1, 0, 0}, 0)
    
    trajectory = sim.addDrawingObject(sim.drawing_linestrip, 5, 0, -1, 9000, {1, 0, 0})
    
    -- tractor body handle
    tractorBody = sim.getObjectHandle("Shape2")
    
    -- Update current position and heading 
    tractorPos = sim.getObjectPosition(tractorBody, -1)
    currentOrientationEuler = sim.getObjectOrientation(tractorBody,sim.handle_world)
    atmp = currentOrientationEuler[1];
    btmp = currentOrientationEuler[2];
    ctmp = currentOrientationEuler[3]; 
    yawtmp,pitchtmp, rolltmp=sim.alphaBetaGammaToYawPitchRoll(atmp,btmp,ctmp)      
    -- Update current tractor pose
    currentPose.x = tractorPos[1]
    currentPose.y = tractorPos[2]
    currentPose.heading = yawtmp
    ----------------------------------------
    
    -- gate posts handle
    gateLeft=sim.getObject("/Cylinder[2]")
    gateRight=sim.getObject("/Cylinder[1]")
    
    
    ---------------[[ Calculate Gate Position ]]---------------
    position0 = sim.getObjectPosition(gateLeft, -1)
    position1 = sim.getObjectPosition(gateRight, -1)
    
    targetPose.x = (position0[1] + position1[1]) / 2
    targetPose.y = (position0[2] + position1[2]) / 2
   
  
    --------------[[ Calculate Gate Orientation ]]-------------
    -- Calculate direction vector for gate orientation
    local gateDirection = {position1[1] - position0[1], position1[2] - position0[2]}
    
    -- Calculate the perpendicular direction vector
    local perpendicularDirection = {-gateDirection[2], gateDirection[1]}
    
    -- Calculate gate orientation (phi) using the perpendicular direction
    local gateOrientation = math.atan2(perpendicularDirection[1], perpendicularDirection[2])
    
    gateOrientation = gateOrientation + math.pi / 2
    
    targetPose.phi = gateOrientation
    ----------------------------------------------------------
    
    -- Calculate initial distance e
    local dx = targetPose.x - currentPose.x
    local dy = targetPose.y - currentPose.y
    distance = math.sqrt(dx*dx + dy*dy)
    
    -- relative orientation of the car with respect to the target pose: 
    phi = currentPose.heading - targetPose.phi
        
    -- the angle to the target pose from the tractor's current position:
    theta = math.atan2(targetPose.y-currentPose.y,targetPose.x-currentPose.x)-targetPose.phi
    
    -- normalize theta to be within [-pi,pi]:
    theta = math.atan2(math.sin(theta), math.cos(theta))
        
    -- the angle between the car's current orientation 
    -- and the direction towards the target pose:
    alpha = theta - phi
    
    -- normalize alpha to be within [-pi,pi]:
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))    
    angdiff = currentPose.heading-targetPose.phi
    angdiff = math.atan2(math.sin(angdiff), math.cos(angdiff))  
end


function sysCall_actuation()    
    -- Non linear controller
    if (math.abs(alpha) <1e-8) then
        wRef = k*alpha+gamma*math.cos(alpha)*(alpha+h*theta);
        uRef = gamma*math.cos(alpha)*distance;
    else
        wRef = k*alpha+gamma*math.cos(alpha)*math.sin(alpha)/alpha*(alpha+h*theta);
        uRef = gamma*math.cos(alpha)*distance;
    end
    --------------------------------------------------------
    
    if (turnRateControl == 0) then
        wRef = 0
    end

    if (velocityControl == 0) and (turnRateControl==1) then
        angdiff = math.atan2(math.sin(angdiff),math.cos(angdiff))
        if (angdiff) > 0 then
            wRef=-0.1
        elseif (angdiff) < 0 then 
            wRef=0.1
        else
            wRef=0
        end
        uRef = 0
    end
    
    G_v = 0.2159  -- Plant constant for velocity
    G_w = 0.238   -- Average plant constant for turning rate (e.g., (0.221 + 0.256) / 2)

    -- PI controller for turning rate
    if (turnRateControl == 1) or (turnRateControl == 0) then
     -- PI parameters for the turning rate
        Krot_p = 0.3;
        Krot_i = 1.0;
        intRot = intRot + (wRef-wMes)*50/1000; -- integrator dt=50ms  (wMes: measurement of turning rate)
        wFF=wRef/G_w;
        uOmega=Krot_p*(wRef-wMes)+Krot_i*intRot + wFF; 
    end 
   -------------------------------------
   
    -- PI controller for velocity
    if (velocityControl == 1) then
     -- PI parameters for the velocity
        Kvel_p = 0.5;
        Kvel_i = 1.5;
        intVel = intVel+(uRef-uMes)*50/1000; -- integrator dt=50ms (uMes: measurement of velocity)
        uFF = uRef/G_v;
        uV = Kvel_p*(uRef-uMes)+Kvel_i*intVel + uFF;
    else
        uV=0;
    end
    ------- Testing feedforward values -------
    --uOmega=0;
    --uV=0;
    ------------------------------------------
    -- send left and right motor velocity 
    sim.setJointTargetVelocity(motorLeft1,-(uV-uOmega))
    sim.setJointTargetVelocity(motorLeft2,-(uV-uOmega))
    sim.setJointTargetVelocity(motorRight1,-(uV+uOmega))
    sim.setJointTargetVelocity(motorRight2,-(uV+uOmega))
    --sim.setJointTargetVelocity(motorLeft1,-0.9)
    --sim.setJointTargetVelocity(motorLeft2,-0.9)
    --sim.setJointTargetVelocity(motorRight1,-1.2)
    --sim.setJointTargetVelocity(motorRight2,-1.2)

end
------------------------------------------------------------

function sysCall_sensing()
    -- Update current position and heading 
    tractorPos = sim.getObjectPosition(tractorBody, -1)
    currentOrientationEuler = sim.getObjectOrientation(tractorBody,sim.handle_world)
    atmp = currentOrientationEuler[1];
    btmp = currentOrientationEuler[2];
    ctmp = currentOrientationEuler[3]; 
    yawtmp,pitchtmp, rolltmp=sim.alphaBetaGammaToYawPitchRoll(atmp,btmp,ctmp)      
    -- Update current tractor pose
    currentPose.x = tractorPos[1]
    currentPose.y = tractorPos[2]
    currentPose.heading = yawtmp
    -----------------------------------------   
    --- For non-linear controller -----------
    -- update distance e
    local dx = targetPose.x - currentPose.x
    local dy = targetPose.y - currentPose.y
    distance = math.sqrt(dx*dx + dy*dy)    
    -- relative orientation of the car with respect to the target pose: 
    phi = currentPose.heading - targetPose.phi
    -- the angle to the target pose from the tractor's current position:
    theta = math.atan2(targetPose.y-currentPose.y,targetPose.x-currentPose.x)-targetPose.phi
    -- normalize theta to be within [-pi,pi]:
    theta = math.atan2(math.sin(theta), math.cos(theta))
    -- the angle between the car's current orientation 
    -- and the direction towards the target pose:
    alpha = theta - phi
    -- normalize alpha to be within [-pi,pi]:
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))

   

    -- read turning rate and velocity
    velMes, rateMes = sim.getVelocity(tractorBody)
    
    -- v is a table containing the linear velocity components along the x, y, and z axes
    --local vx = velMes[1]
    --local vy = velMes[2]
    --local vz = velMes[3]
    
    -- omega is a table containing the angular velocity components around the x, y, and z axes
    local omegaX = rateMes[1]
    local omegaY = rateMes[2]
    local omegaZ = rateMes[3]  -- This represents the turning rate around the z-axis

    --uMes = math.sqrt(vx^2 + vy^2)
    uMes =  velMes[1]*math.cos(currentPose.heading)+velMes[2]*math.sin(currentPose.heading); 
    wMes = omegaZ
    angdiff = currentPose.heading-targetPose.phi
    angdiff = math.atan2(math.sin(angdiff), math.cos(angdiff))  
    if (distance <= 0.1) and (turnRateControl == 1) then
        uRef = 0;
        velocityControl = 0;
        angdiff = currentPose.heading-targetPose.phi
        angdiff = math.atan2(math.sin(angdiff),math.cos(angdiff))
        if (angdiff) > 0 then 
            wRef=-0.1 
        elseif (angdiff) < 0 then
            wRef=0.1
        else
            wRef=0
        end
        if (math.abs(angdiff) <= 2*math.pi/180) then
            wRef = 0;
            turnRateControl = 0;
        end
    end
    print('current pose:',currentPose)
    print('hading / angdiff :',currentPose.heading*180/math.pi, angdiff*180/math.pi)
    sim.setGraphStreamValue(handleGraph, stream1, wRef)
    sim.setGraphStreamValue(handleGraph, stream2, wMes)

    sim.setGraphStreamValue(handleGraph1, stream3, uRef)
    sim.setGraphStreamValue(handleGraph1, stream4, uMes)
    print("uMes",uMes,"Velocty",math.sqrt(velMes[1]*velMes[1]+velMes[2]*velMes[2]),
    "headin",currentPose.heading*180/math.pi,"uRef",uRef);
    sim.addDrawingObjectItem(trajectory, tractorPos)
    
    -------------------------------------
    file:write(string.format('time: %.3f [s]', sim.getSimulationTime() + sim.getSimulationTimeStep())) -- time stamp
    file:write(string.format('wRef: %.3f [rad/s], ', wRef)) -- Reference turning rate
    file:write(string.format('wMes: %.3f [rad/s], ', wMes)) -- Measured turning rate
    file:write(string.format('uRef: %.3f [m/s], ', uRef)) -- Reference velocity
    file:write(string.format('uMes: %.3f [m/s]\n', uMes)) -- Measured velocity
    -------------------------------------
    accuracy:write(string.format("time: %.3f [s], %f, %f\n", sim.getSimulationTime() + sim.getSimulationTimeStep(), tractorPos[1], tractorPos[2]))
    -------------------------------------
end

function sysCall_cleanup()
    -- do some clean-up here
    file:close()  -- Ensure this is called in the sysCall_cleanup function to close the file at the end of the simulation
    accuracy:close()
end


