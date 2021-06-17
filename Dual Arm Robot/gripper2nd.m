function gripper2nd (clientID,closing,j3,j4)
sim=remApi('remoteApi');
[r,p3]=sim.simxGetJointPosition(clientID,j3,sim.simx_opmode_blocking);
[r,p4]=sim.simxGetJointPosition(clientID,j4,sim.simx_opmode_blocking);
if (closing==1)
    if (p3 <(p4-0.008))
        sim.simxSetJointTargetVelocity (clientID,j3,-0.01,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity (clientID,j4,-0.04,sim.simx_opmode_blocking);
    else
        sim.simxSetJointTargetVelocity (clientID,j3,-0.04,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity (clientID,j4,-0.04,sim.simx_opmode_blocking);
    end
else
    if (p3<p4)
        sim.simxSetJointTargetVelocity (clientID,j3,0.04,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity (clientID,j4,0.02,sim.simx_opmode_blocking);
    else
        sim.simxSetJointTargetVelocity (clientID,j3,0.02,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity (clientID,j4,0.04,sim.simx_opmode_blocking);
    end
end

end