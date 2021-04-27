% Feel free to add code anywhere you find necessary to solve the task.
% as a guide, we have provide recommendation where to add code

% The code is for the implementation of the task in which tracking of the robot has been done using camera. We
% need to control the camera servo motors so that the robot always remains in the center of the camera.


%% DO NOT CHANGE ANYTHING HERE - Setting up the remote api
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

% connection status
connected = false;

% ultrasonic sensor handles
rangefinder = zeros(1, 6);

% Object handles
yawMotor = 0;
pitchMotor = 0;
camera = 0;

kp_pitch=1;

kp_yaw=1;

if (clientID>-1)
    
    connected = true;
    disp('Connected to remote API server');
    
    % get camera system handles
    [~,camera]=vrep.simxGetObjectHandle(clientID,'VisionSensor',vrep.simx_opmode_blocking);
    [~,pitchMotor]=vrep.simxGetObjectHandle(clientID,'Pitch',vrep.simx_opmode_blocking);
    [~,yawMotor]=vrep.simxGetObjectHandle(clientID,'Yaw',vrep.simx_opmode_blocking);
    [~,body]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    
    % initialize camera
    [~, resolution, image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_streaming);
    
    
end

if (connected)
    clf;
    i=0;
    clock1=tic;
    while 1
        [~, resolution, image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_streaming);
        [~, resolution, image]=vrep.simxGetVisionSensorImage2(clientID,camera,1,vrep.simx_opmode_buffer);
        t=toc(clock1);
        if i>1
            last_pos_t=pos_t;
        end
        [returnCode, posnew]=vrep.simxGetObjectPosition(clientID,body,-1,vrep.simx_opmode_blocking);
        pos_t = [posnew(1),posnew(2),t];
        if i>1
            target_x= 16
            target_y= 16
            
            [current_x,current_y]=find_matrix_center(image);
            if current_x==0 & current_y==0
                [~]=vrep.simxSetJointTargetVelocity(clientID, pitchMotor, 0, vrep.simx_opmode_blocking);
                [~]=vrep.simxSetJointTargetVelocity(clientID, yawMotor, 0, vrep.simx_opmode_blocking);
            else
                error_x=(16-current_x)/32; %normalising the error
                error_y=(16-current_y)/32; %normalising the error
                v_pitch= -kp_pitch*error_y;
                v_yaw= kp_yaw*error_x;
                [~]=vrep.simxSetJointTargetVelocity(clientID, pitchMotor,v_pitch, vrep.simx_opmode_blocking);
                [~]=vrep.simxSetJointTargetVelocity(clientID, yawMotor,v_yaw, vrep.simx_opmode_blocking);
                
            end
            
        end
        if (t>90)
            break;
        end
        if i==2
            plot3(pos_t(1),pos_t(2),pos_t(3),'o');
        end
        if i>1
            plot3([pos_t(1),last_pos_t(1)],[pos_t(2),last_pos_t(2)],[pos_t(3),last_pos_t(3)],'r');
            hold on; grid on;
            zlabel("Time")
            xlabel("X-Coordinate");
            ylabel("Y-Coordinate");
        end
        i=i+1;
    end
    plot3(pos_t(1),pos_t(2),pos_t(3),'x');
        
    % stop camera system motors
    [~]=vrep.simxSetJointTargetVelocity(clientID, pitchMotor, 0, vrep.simx_opmode_blocking);
    [~]=vrep.simxSetJointTargetVelocity(clientID, yawMotor, 0, vrep.simx_opmode_blocking);
    
    % Now close the connection to V-REP:
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!

%finding the center of the matrix using the weighted mean method
function [x_center,y_center]= find_matrix_center(image_matrix)
sum_col=sum(image_matrix,1); %Sum of different columns
sum_row=sum(image_matrix,2); %Sum of different rows
sum_matrix=sum(image_matrix, 'all')
weighted_sum_y=0;
weighted_sum_x=0;
for j=1:length(sum_col)
    weighted_sum_x=weighted_sum_x+j*sum_col(j)
end
for i=1:length(sum_row)
    weighted_sum_y=weighted_sum_y+i*sum_row(i)
end
%if all the elements of the matrix are zero then the center is 16,16
if sum_matrix==0
    x_center=16;
    y_center=16;
else
    x_center=round(weighted_sum_x/sum_matrix)
    y_center=round(weighted_sum_y/sum_matrix)
end
end

