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
    
    % Object handle
    [~,bot]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
    [~, pos]=vrep.simxGetObjectPosition(clientID,bot,-1,vrep.simx_opmode_streaming);
    
    % GPS signal
    [gps_err,gps_signal]=vrep.simxReadStringStream(clientID,'gps_data',vrep.simx_opmode_streaming); 
end
% call figure
%--------------------------------------------------------------------
environment(); %uncomment this line to plot in the map
%--------------------------------------------------------------------
line = line(nan, nan, 'color', 'red');
 
 if (connected)   
     
    % read GPS position
    [gps_err,gps_signal]=vrep.simxReadStringStream(clientID,'gps_data',vrep.simx_opmode_buffer);

    if gps_err == vrep.simx_return_ok
        [gps_buffer]= vrep.simxUnpackFloats(gps_signal);
    end 
    % getting initial coordinates for initial correction
    [~, pos]=vrep.simxGetObjectPosition(clientID,bot,-1,vrep.simx_opmode_blocking);
     xi = pos(1); %initial x position
     yi = pos(2); % initial y position
     % correction parameters
     x_corr= (xi-gps_buffer(1)); 
     y_corr= (yi-gps_buffer(2));
     
    while 1
        % read GPS position
        [gps_err,gps_signal]=vrep.simxReadStringStream(clientID,'gps_data',vrep.simx_opmode_buffer);
        
        if gps_err == vrep.simx_return_ok
            [gps_buffer]= vrep.simxUnpackFloats(gps_signal);
            disp(gps_buffer); % sequence on (x, y, z,) positions
        end
        
        % Retrieves the simulation time of the last fetched command 
        [cmdTime] = vrep.simxGetLastCmdTime(clientID); % TIME IF YOU NEED IT

        % getting x and y coordinates
        for i= 1:3:length(gps_buffer)
            xcd = gps_buffer(i)+x_corr;
            ycd= gps_buffer(i+1)+y_corr;
            
        % plot in real-time
        x = get(line, 'xData');
        y  = get(line, 'yData');
        
       % --------------------------------------------------------------------------------
        x = [x, xcd];   % change xcd to any variable you want plotted
        y = [y, ycd]; % change ycd to any variable you want plotted
        %---------------------------------------------------------------------------------
        
        set(line, 'xData', x, 'yData', y);
        end
        pause(0.1)
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