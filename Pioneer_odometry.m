% Feel free to add code anywhere you find necessary to solve the task.
% as a guide, we have provide recommendation where to add code
clear all, clc, clf
%% DO NOT CHANGE ANYTHING HERE - Setting up the remote api
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections
%clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
clientID=vrep.simxStart('127.0.0.1',19997,true,true,5000,1);

% connection status
connected = false;

% robot parameters
d = 0.1950; % wheel radius
r = d/2; % wheel radius
T = 0.3310;% wheel track

if (clientID>-1)
    
    connected = true;
    disp('Connected to remote API server');
    
    % start simulation
    e = vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    
    % get object handles
    [~,leftMotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',vrep.simx_opmode_blocking);
    [~,rightMotor]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',vrep.simx_opmode_blocking);
    [~,bot]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    
    % initialize motor position reading (encoder)
    [~, rotLM]=vrep.simxGetJointPosition(clientID, leftMotor, vrep.simx_opmode_streaming);
    [~, rotRM]=vrep.simxGetJointPosition(clientID, rightMotor, vrep.simx_opmode_streaming);
    
    [~, pos]=vrep.simxGetObjectPosition(clientID,bot,-1,vrep.simx_opmode_streaming);
    [~, orient]=vrep.simxGetObjectOrientation(clientID,bot,-1,vrep.simx_opmode_streaming);
    
    
end
% call figure
%--------------------------------------------------------------------
environment(); %uncomment this line to plot in the map
%--------------------------------------------------------------------
line = line(nan, nan, 'color', 'red');

if (connected)
    [~, pos]=vrep.simxGetObjectPosition(clientID,bot,-1,vrep.simx_opmode_blocking);
    [~, orient]=vrep.simxGetObjectOrientation(clientID,bot,-1,vrep.simx_opmode_blocking);
    %taking initial position and orientation of robot
    xi = pos(1);
    yi = pos(2);
    thi = orient(3);
    i=1;  
    while 1 
        % get motor angular position (encoder emulator)
        [~, rotLM]=vrep.simxGetJointPosition(clientID, leftMotor, vrep.simx_opmode_buffer );
        [~, rotRM]=vrep.simxGetJointPosition(clientID, rightMotor, vrep.simx_opmode_buffer );
        
        % converting from (-pi,pi) sensor reading to (0,2*pi) 
        if(rotLM<0)
            rotLM= rotLM+2*pi;
        end
        
        if(rotRM<0)
            rotRM= rotRM+2*pi;
        end
        
        %wheel angular position in degrees and radians
        rotLRdeg (i,:)= [rotLM*180/pi, rotRM*180/pi];
        rotLRrad (i,:)= [rotLM, rotRM];
        
        % Retrieves the simulation time of the last fetched command
        [cmdTime] = vrep.simxGetLastCmdTime(clientID); % TIME IF YOU NEED IT
        
        if(i>1)  % executing on second loop run. In first loop run, getting the initial sensor readings             
            
            % Left wheel
            if(rotLRrad(i,1)<rotLRrad((i-1),1))  % detecting clockwise (forward motion) zero crossing 
                delta_rotLMrad=2*pi-rotLRrad((i-1),1)+rotLRrad(i,1);
            elseif(rotLRrad(i,1)>rotLRrad((i-1),1)) & abs(rotLRrad(i,1)-rotLRrad((i-1),1))>pi  % detecting anti-clockwise (backward motion) zero crossing
                delta_rotLMrad = -2*pi-rotLRrad((i-1),1)+rotLRrad(i,1);
            else delta_rotLMrad = rotLRrad(i,1)-rotLRrad((i-1),1);  % no zero crossing 
            end
            
            % Right wheel
            if(rotLRrad(i,2)<rotLRrad((i-1),2))  % detecting clockwise (forward motion) zero crossing
                delta_rotRMrad=2*pi-rotLRrad((i-1),2)+rotLRrad(i,2);
            elseif(rotLRrad(i,2)>rotLRrad((i-1),2)) & abs(rotLRrad(i,2)-rotLRrad((i-1),2))>pi  % detecting anti-clockwise (backward motion) zero crossing
                delta_rotRMrad= -2*pi-rotLRrad((i-1),2)+rotLRrad(i,2);
            else
                delta_rotRMrad= rotLRrad(i,2)-rotLRrad((i-1),2); % no zero crossing
            end
            % kinematics equation
            % distance travelled by right/left wheel ('+' means along---> + x axis of robot frame)
            Sr = delta_rotRMrad * r; 
            Sl = delta_rotLMrad * r;
            dth = (Sr-Sl)/T;  %angle rotated in robot frame z-axis("+" means clockwise)
            
            % adding change in coordinates to initial coordinates to get
            % updated coordinates
            cordn = [xi; yi; thi] + [((Sr+Sl)/2)*cos(thi + (dth/2)); ((Sr+Sl)/2)*sin(thi + (dth/2)); dth ]; 
            
            % plot in real-time
            x = get(line, 'xData');
            y  = get(line, 'yData');
            
            %---------------------------------------------------------------------------------
            x = [x, cordn(1,:)];      % change cordn(1,:) to any variable you want plotted
            y = [y, cordn(2,:)];     % change cordn(2,:) to any variable you want plotted
            %---------------------------------------------------------------------------------
            
            set(line, 'xData', x, 'yData', y);
            
            % updating initial coordinates
            xi = cordn(1,1);
            yi = cordn(2,1);
            thi = cordn(3,1);
            pause(0.01);
        end
        i=i+1; % index increment of wheel positions vector
    end
    
    % stop simulation
    [~]=vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot);
    pause(5);
    
    % Now close the connection to V-REP:
    vrep.simxFinish(clientID);
else
    disp('Failed connecting to remote API server');
end
vrep.delete(); % call the destructor!