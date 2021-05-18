% Feel free to add code anywhere you find necessary to solve the task.
% as a guide, we have provide recommendation where to add code
clear all, clc

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

t1 = 0;

arr_a = [];
arr_dt = [];
arr_omega = [];
arr_theta = [];
arr_v = [];
arr_vx = [];
arr_vy = [];
arr_x = [];
arr_y = [];
arr_t = [];
arr_ang = [];

if (clientID>-1)
    
    connected = true;
    disp('Connected to remote API server');
    
    % start simulation
    e = vrep.simxStartSimulation(clientID,vrep.simx_opmode_blocking);
    
    % IMU signals
    [gyro_err,gyro_signal]=vrep.simxReadStringStream(clientID,'gyro_data',vrep.simx_opmode_streaming);
    [accel_err,accel_signal]=vrep.simxReadStringStream(clientID,'accel_data',vrep.simx_opmode_streaming);
    
end

% call figure
%--------------------------------------------------------------------
environment(); %uncomment this line to plot in the map

%--------------------------------------------------------------------
line = line(nan, nan, 'color', 'red');

[returnCode, bot]=vrep.simxGetObjectHandle(clientID,'Pioneer_p3dx',vrep.simx_opmode_blocking);
[returnCode, posnew]=vrep.simxGetObjectPosition(clientID, bot,-1,vrep.simx_opmode_blocking);
[returnCode, bot_angles]=vrep.simxGetObjectOrientation(clientID,bot,-1,vrep.simx_opmode_blocking);

 if (connected)   
    while true % CHANGE THIS LINE TO 'while loop'

        % read gyroscope and accelerometer data
        [gyro_err,gyro_signal]=vrep.simxReadStringStream(clientID,'gyro_data',vrep.simx_opmode_buffer);
        [accel_err,accel_signal]=vrep.simxReadStringStream(clientID,'accel_data',vrep.simx_opmode_buffer);
        
        % Gyroscope
        if gyro_err == vrep.simx_return_ok
           [gyro_buffer]= vrep.simxUnpackFloats(gyro_signal);
%          disp(gyro_buffer) % sequence on (x, y, z,) angular velocities
        end
        
        % Accelerometer
        if accel_err == vrep.simx_return_ok
           [accel_buffer]= vrep.simxUnpackFloats(accel_signal);
%          disp(accel_buffer) % sequence on (x, y, z,) accelerations
        end
        
        % Retrieves the simulation time of the last fetched command 
        [cmdTime] = vrep.simxGetLastCmdTime(clientID); % TIME IF YOU NEED IT
        
        % Calculating the time intervals for the cycle
        dt = (cmdTime - t1)/1000
        t1 = cmdTime;
        
        % Taking diferent time values into an array
        arr_dt = [arr_dt,dt];
        
        % averaging the acceleration components along x & y - axis and the
        % gyroscopic readings
        
        i = 1;
        accel_sum = [0 0];
        omega_sum = 0;
        while i<length(accel_buffer)/3
            accel_sum(1,1) = accel_sum(1,1) + accel_buffer(1,1+(i-1)*3);
            accel_sum(1,2) = accel_sum(1,2) + accel_buffer(1,2+(i-1)*3);
            omega_sum = omega_sum + gyro_buffer(1, 3+(i-1)*3);
            i = i+1;
        end
        
        accel_curr = [accel_sum(1,1)/(length(accel_buffer)/3) accel_sum(1,2)/(length(accel_buffer)/3)];    
        omega_curr = omega_sum/(length(gyro_buffer)/3);
        accel_theta = atan(accel_curr(1,2)/accel_curr(1,1));
        arr_t = [arr_t; t1/1000];
        
        % Finding the angle of rotation from the gyroscopic readings 
        %(error corrections included)
        
        if omega_curr <= -0.2
            theta = omega_curr*dt - 0.07;
        end
        
        if omega_curr >= 0.2
            theta = omega_curr*dt + 0.006;
        end
        
        if omega_curr > -0.3 & omega_curr < 0.2
            if omega_curr < 0
            theta = (omega_curr)*dt + 0.0003;
            end
            if omega_curr > 0
            theta = omega_curr*dt - 0.0002
            end
        end
        
        bot_angles(1,3) = (bot_angles(1,3) + theta);
        
        % finding the magnitude of acceleration
        accel_total = sqrt(accel_curr(1,1)^2 + accel_curr(1,2)^2)
        
        %arrays for acceleration components 
        arr_omega = [arr_omega; omega_curr];
        arr_theta = [arr_theta; theta];        
        arr_a = [arr_a; accel_total];
        arr_omega = [arr_omega; omega_curr];
        
        %Finding the overall velocity
        v_upd = accel_total*dt + 0.04;
        
        %Finding the position co-ordinates
        pos_x = posnew(1,1) + (v_upd*dt + 0.0004)*cos(bot_angles(1,3) + theta/2);
        pos_y = posnew(1,2) + (v_upd*dt + 0.0004)*sin(bot_angles(1,3) + theta/2);
        
        %Finding the velocity components
        v_upd_x = (v_upd*dt)*cos(bot_angles(1,3) + theta/2);
        v_upd_y = (v_upd*dt)*sin(bot_angles(1,3) + theta/2);
        arr_ang = [arr_ang; accel_theta]; 
       
        %Position Updation
        posnew(1,1) = pos_x;
        posnew(1,2) = pos_y;
        
        arr_vx = [arr_vx; v_upd_x];
        arr_vy = [arr_vy; v_upd_y];
        arr_x = [arr_x; pos_x];
        arr_y = [arr_y; pos_y];
        
        % ADD CODE HERE
        
        % plot in real-time
        x = get(line, 'xData');
        y  = get(line, 'yData');
        
        %---------------------------------------------------------------------------------
        x = [x, pos_x];   % change n to any variable you want plotted
        y = [y, pos_y]; % change m to any variable you want plotted
        %---------------------------------------------------------------------------------
        
        set(line, 'xData', x, 'yData', y);
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