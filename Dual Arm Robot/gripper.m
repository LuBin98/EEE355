function gripper (clientID,closing,j1,j2)
sim=remApi('remoteApi');
[r,p1]=sim.simxGetJointPosition(clientID,j1,sim.simx_opmode_blocking);
[r,p2]=sim.simxGetJointPosition(clientID,j2,sim.simx_opmode_blocking);
if (closing==1)
    if (p1 <(p2-0.008))
        sim.simxSetJointTargetVelocity (clientID,j1,-0.01,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity (clientID,j2,-0.04,sim.simx_opmode_blocking);
    else
        sim.simxSetJointTargetVelocity (clientID,j1,-0.04,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity (clientID,j2,-0.04,sim.simx_opmode_blocking);
    end
else
    if (p1<p2)
        sim.simxSetJointTargetVelocity (clientID,j1,0.04,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity (clientID,j2,0.02,sim.simx_opmode_blocking);
    else
        sim.simxSetJointTargetVelocity (clientID,j1,0.02,sim.simx_opmode_blocking);
        sim.simxSetJointTargetVelocity (clientID,j2,0.04,sim.simx_opmode_blocking);
    end
end

end