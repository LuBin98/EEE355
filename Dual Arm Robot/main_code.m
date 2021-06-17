clear all
close all
clc
origine_table=[-0.18,1.7750,0.63];
cube_dimensions=[0.05,0.07,0.05];
width_offset=0.02;
length_offset=0.05;
end_test=0;
end_test_0=0;
sim=remApi('remoteApi');
sim.simxFinish(-1);
clientID=sim.simxStart('127.0.0.1',19999,true,true,5000,5);
if (clientID >-1)
    disp ('connected to remote API server');
    %object handles
    [res,j1]=sim.simxGetObjectHandle(clientID,'ROBOTIQ_85_active1',sim.simx_opmode_blocking);
    [res,j2]=sim.simxGetObjectHandle(clientID,'ROBOTIQ_85_active2',sim.simx_opmode_blocking);
    [res,kuka_target]=sim.simxGetObjectHandle(clientID,'target',sim.simx_opmode_blocking); %KUKA_KR_16_target
    [res,Proximity_sensor]=sim.simxGetObjectHandle(clientID,'Proximity_sensor',sim.simx_opmode_blocking);

    [res,j3]=sim.simxGetObjectHandle(clientID,'ROBOTIQ_85_active1#0',sim.simx_opmode_blocking);
    [res,j4]=sim.simxGetObjectHandle(clientID,'ROBOTIQ_85_active2#0',sim.simx_opmode_blocking);
    [res,kuka_target_0]=sim.simxGetObjectHandle(clientID,'target#0',sim.simx_opmode_blocking); %KUKA_KR_16_target
    [res,Proximity_sensor_0]=sim.simxGetObjectHandle(clientID,'Proximity_sensor#0',sim.simx_opmode_blocking);
    
    %let's define now the target positions needed
    
    fposition3=[-0.12,-0.3,0.75,0,0,0]; % above pickup position
    fposition4=[-0.12,-0.3,0.63,0,0,0]; % pickup position
    fposition5=[0,0.2,0.75,0,0,0];      % above place position
    fposition6=[0,0.2,0.675,0,0,0];      % place position
    
    %let's define now the target positions needed
    
    fposition9=[0.017,1.009,0.70,0,0,0]; % above pickup position
    fposition10=[0.017,1.009,0.63,0,0,0]; % pickup position
    fposition11=[-0.18,1.7750,0.72,0,0,0];      % above place position
    fposition12=[-0.18,1.7750,0.63,0,0,0];      % place position
 
    gripper (clientID,0,j1,j2); %open gripper
    gripper2nd (clientID,0,j3,j4); %open gripper
    moveL(clientID, kuka_target ,fposition3,2); 
    moveL(clientID, kuka_target_0 ,fposition9,2); 
    pause(1.5);
    
        while (end_test==0 && end_test_0==0)
            [res,PSensor_distance, detectedPoint]= sim.simxReadProximitySensor(clientID, Proximity_sensor,sim.simx_opmode_blocking);
            [res,PSensor_distance_0, detectedPoint]= sim.simxReadProximitySensor(clientID, Proximity_sensor_0,sim.simx_opmode_blocking);
            
            if (PSensor_distance > 0)
                moveL(clientID, kuka_target ,fposition4,2);
                gripper (clientID,1,j1,j2);pause(1); %close gripper and pickup the cube
                moveL(clientID, kuka_target ,fposition3,2);
                moveL(clientID, kuka_target ,fposition5,2);
                moveL(clientID, kuka_target ,fposition6,2);
                gripper (clientID,0,j1,j2);pause(0.5);
                moveL(clientID, kuka_target ,fposition5,2);
                moveL(clientID, kuka_target ,fposition3,2);
            end
      
            if (PSensor_distance_0 > 0)
                moveL(clientID, kuka_target_0 ,fposition10,2);
                gripper2nd (clientID,1,j3,j4);pause(1.5); %close gripper and pickup the cube
                moveL(clientID, kuka_target_0 ,fposition9,2);
                moveL(clientID, kuka_target_0 ,fposition11,2);
                moveL(clientID, kuka_target_0 ,fposition12,2);
                gripper2nd (clientID,0,j3,j4);pause(0.5);
                moveL(clientID, kuka_target_0 ,fposition11,2);
                moveL(clientID, kuka_target_0 ,fposition9,2);
                %refresh the place position
                [end_test_0,fposition12,fposition11,fposition9]=pick_and_place(origine_table,2,2,2,cube_dimensions,width_offset,length_offset,fposition12,fposition11,fposition9);
            end
        end
         
sim.delete(); % call the destructor 
disp('program ended');
end     