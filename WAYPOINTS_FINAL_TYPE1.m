vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
connected = false;

if (clientID>-1)
    connected = true;
    disp('Connected to remote API server');
end

% Vehicle parameters
a = 0.158516;       % COM to any front/back wheels [m]
b = 0.228020;       % COM to any right/left wheels [m]
R = 0.050369;     % Wheel radius [m]

if(connected)
    % handles to wheel motors
    [returnCode,wheelFL]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fl',vrep.simx_opmode_blocking);
    [returnCode,wheelFR]=vrep.simxGetObjectHandle(clientID,'rollingJoint_fr',vrep.simx_opmode_blocking);
    [returnCode,wheelRR]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rr',vrep.simx_opmode_blocking);
    [returnCode,wheelRL]=vrep.simxGetObjectHandle(clientID,'rollingJoint_rl',vrep.simx_opmode_blocking);
    
    % handle to robot body reference frame
    [returnCode,bodyFrame]=vrep.simxGetObjectHandle(clientID,'body_frame',vrep.simx_opmode_blocking);
end


%kp, kd, ki for x axis
kp_x=1;
kd_x=0.01;
ki_x=0.001;

%kp, kd for y axis
kp_y=1;
kd_y=0.01;
ki_y=0.001;

%kp, kd for gamma
kp_g=0.01;
kd_g=0.00;
ki_g=0.000;

%% To Do
if(connected)
    pause(2);
    waypoints={[1,0,0],[0,1,0],[1,1,0],[0,0,0],"end"};
    waypointIndex=1;
    tf = isequal(waypoints{waypointIndex},"end");
    
    jacobian_inv=(1/R)*[1 1 -a-b;-1 1 a+b;-1 1 -a-b;1 1 a+b];
    
    while ~tf
        destination=waypoints{waypointIndex}; %new waypoinit
        posf=destination;
        
        [returnCode, posnew]=vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
        
        %for orientation correction
        [returnCode, bot_angles]=vrep.simxGetObjectOrientation(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
        bot_angles=(bot_angles-(pi/2))*180/pi;
        
        [posf_l]=transform_posf(posf,posnew,bot_angles);
        
        angle_correction=find_angle_correction(posf_l);
        last_error_g=angle_correction;
        
        error_x=posf_l(1,1);
        error_y=posf_l(2,1);
        
        distance=sqrt(((error_x)^2)+((error_y)^2));
        last_error_x=error_x;
        last_error_y=error_y;
        integral_x=0;
        integral_y=0;
        clock1=tic;
        t=toc(clock1);
        while abs(angle_correction)>1 | distance>0.08
            
            %keeping the orientation of the robot constant
            
            if abs(angle_correction)>1
                dt=toc(clock1)-t;
                t=toc(clock1);
                error_g=angle_correction;
                derivative_g=(error_g- last_error_g)/dt;
                
                g_vel=kp_g*error_g+ kd_g*derivative_g;
                
                x_vel=0;
                y_vel=0;
                [wheel_vel]=[jacobian_inv]*[x_vel; y_vel; g_vel];
                
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFL,wheel_vel(1,1), vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFR,wheel_vel(2,1), vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRL,wheel_vel(3,1), vrep.simx_opmode_blocking);
                [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRR,wheel_vel(4,1), vrep.simx_opmode_blocking);
                
                %pause(Ts);
                
                [returnCode, posnew]=vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
                
                %for orientation correction
                [returnCode, bot_angles]=vrep.simxGetObjectOrientation(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
                bot_angles=(bot_angles-(pi/2))*180/pi;
                
                [posf_l]=transform_posf(posf,posnew,bot_angles);
                angle_correction=find_angle_correction(posf_l);
                last_error_g=error_g;
            end
            if abs(angle_correction)<1
                disp("entered this loop");
                while distance>0.08
                    dt=toc(clock1)-t;
                    t=toc(clock1);
                    %PID x velocity for the robot
                    derivative_x= (error_x-last_error_x)/dt;
                    integral_x=integral_x+error_x*dt;
                    
                    x_vel= kp_x*error_x+ kd_x*derivative_x+ ki_x*integral_x;
                    
                    %PID y velocity for the robot
                    derivative_y= (error_y-last_error_y)/dt;
                    integral_y=integral_y+error_y*dt;
                    
                    y_vel= kp_y*error_y+ kd_y*derivative_y+ ki_y*integral_y;
                    g_vel=0;
                    
                    [wheel_vel]=[jacobian_inv]*[x_vel; y_vel; g_vel];
                    
                    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFL,wheel_vel(1,1), vrep.simx_opmode_blocking);
                    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFR,wheel_vel(2,1), vrep.simx_opmode_blocking);
                    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRL,wheel_vel(3,1), vrep.simx_opmode_blocking);
                    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRR,wheel_vel(4,1), vrep.simx_opmode_blocking);
                    
                    last_error_x=error_x;
                    last_error_y=error_y;
                    
                    
                    [returnCode, posnew]=vrep.simxGetObjectPosition(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
                    [returnCode, bot_angles]=vrep.simxGetObjectOrientation(clientID,bodyFrame,-1,vrep.simx_opmode_blocking);
                    bot_angles=(bot_angles-(pi/2))*180/pi;
                    
                    [posf_l]=transform_posf(posf,posnew,bot_angles);
                    angle_correction=find_angle_correction(posf_l);
                    
                    error_x=posf_l(1,1);
                    error_y=posf_l(2,1);
                    
                    distance=sqrt(((error_x)^2)+((error_y)^2));
                end
                
            end
            
        end
        
        disp("done turning");

        disp("done translating");
        waypointIndex=waypointIndex+1;
        tf = isequal(waypoints{waypointIndex},"end");
    end
    
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFL,0, vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelFR,0, vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRL,0, vrep.simx_opmode_blocking);
    [returnCode]=vrep.simxSetJointTargetVelocity(clientID,wheelRR,0, vrep.simx_opmode_blocking);
    
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end

vrep.delete(); % call the destructor!

function[posf_l]= transform_posf(posf,posnew,bot_angles)

phi=bot_angles(1,3);
T=[cosd(phi) -sind(phi) posnew(1,1); sind(phi) cosd(phi) posnew(1,2); 0 0 1];
posf(1,3)=1;
posf_l= T\posf'

end


function [angle_correction]= find_angle_correction(posf_l)
%angle of the waypoint from the heading
angle_head=atand(posf_l(1,1)/posf_l(2,1));

if posf_l(1,1)>0 & posf_l(2,1)>0
    bot_angles_f=[0 0 (-angle_head)];
end

if posf_l(1,1)<0 & posf_l(2,1)>0
    bot_angles_f=[0 0 abs(angle_head)];
end

if posf_l(1,1)<0 & posf_l(2,1)<0
    bot_angles_f=[0 0 180-abs(angle_head)];
end

if posf_l(1,1)>0 & posf_l(2,1)<0
    bot_angles_f=[0 0 abs(angle_head)-180];
end

angle_correction=bot_angles_f(1,3);
end
