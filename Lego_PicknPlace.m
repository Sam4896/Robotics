clear
clc
myev3 = legoev3('USB');

mymotorb = motor(myev3, 'b');
mymotorc = motor(myev3, 'c');
mymotora = motor(myev3, 'a');

t_sens_1 = touchSensor(myev3,1); %for motor c
t_sens_3 = touchSensor(myev3,3); %for motor b


%% Calibration
start(mymotora);
start(mymotorb);
start(mymotorc);

%% encoder b reset value is determined here because after the while loop, link 3 will be in the air.
%% done so as to avoid collision with station A while turning

kp_b = 0.2;
ki_b = 0.0005;
kd_b = 0.0025;

kp_c = 0.08;
ki_c = 0.0005;
kd_c = 0.003;

gear_ratio_c = 3; %driven/driver
gear_ratio_b = 5; %driven/driver

halfway_setpoint=20*5;

theta_2_max = 88;

%% homming return parameters

[enco_top_b,enco_reset_c,enco_a_initial]=homing(mymotorb,mymotorc,mymotora,t_sens_1,t_sens_3);

%%bringing the arm a bit lower
move_motorb(halfway_setpoint,mymotorb,enco_top_b,kp_b,ki_b,kd_b);
disp("First loop completed");

%% calculation of set points

picking_height=[0,70,0];
picking_angle=[-180,0,-90];
placing_height=[70,0,0];
placing_angle=[0,-90,-180];

prog_cont=1;
path=0; %0 means going to pick up and 1 means going to place
while (prog_cont==1)
    for i=1:3
        if( path==0)
            disp("Going for pickup");
            [theta_1_pick,theta_2_pick] = para_known(picking_height(i),picking_angle(i));
            %             [theta_1_pick,theta_2_pick] = para_calc(); % In case, you
            %             want to input the parameter by yourself. But the loop will
            %             only run for 3 runs.
            set_point_b_pick = (theta_2_max-theta_2_pick)* gear_ratio_b;
            set_point_c_pick = theta_1_pick* gear_ratio_c;
            picking(mymotorb,mymotorc,mymotora,set_point_b_pick,set_point_c_pick,enco_top_b,enco_reset_c,enco_a_initial,kp_b,ki_b,kd_b,kp_c,ki_c,kd_c,halfway_setpoint);
            path=1;
        end
        if (path ==1)
            disp("Going for placing");
            [theta_1_place,theta_2_place] = para_known(placing_height(i),placing_angle(i));
            %             [theta_1_place,theta_2_place] = para_calc(); % In case, you
            %             want to input the parameter by yourself. But the loop will
            %             only run for 3 runs.
            set_point_b_place = (theta_2_max-theta_2_place)* gear_ratio_b;
            set_point_c_place = theta_1_place* gear_ratio_c;
            placing(mymotorb,mymotorc,mymotora,set_point_b_place,set_point_c_place,enco_top_b,enco_reset_c,enco_a_initial,kp_b,ki_b,kd_b,kp_c,ki_c,kd_c,halfway_setpoint);
            path=0;
        end
    end
    prog_cont= 0;
end

function[enco_top_b,enco_reset_c,enco_a_initial] = homing(mymotorb,mymotorc,mymotora,t_sens_1,t_sens_3)
while (readTouch(t_sens_3)==0)
    mymotorb.Speed=-40;
end
enco_top_b=readRotation(mymotorb);
mymotorb.Speed=0;
disp("top reached");

%% using the touch sensor at station A


stationA_reached_c=0;
while (stationA_reached_c==0)
    mymotorc.Speed=50;
    if(readTouch(t_sens_1)==1)%% put the touch sensor command % check for equality sign
        disp("A touched");
        enco_reset_c=readRotation(mymotorc)-10;
        break;
    end
end
mymotorc.Speed=0;

pause(0.2);

% calibration of motor A
for i=1:150
    mymotora.Speed=9;
end

for i=1:150
    mymotora.Speed=-9;
end

mymotora.Speed=0;
mymotorb.Speed=0;
mymotorc.Speed=0;
disp("Calibration done");
enco_a_initial=readRotation(mymotora);
end

function picking(mymotorb,mymotorc,mymotora,set_point_b_pick,set_point_c_pick,enco_top_b,enco_reset_c,enco_a_initial,kp_b,ki_b,kd_b,kp_c,ki_c,kd_c,halfway_setpoint)
% opening the gripper
gripper_opened=0;
while (gripper_opened==0)
    enco_a=readRotation(mymotora);
    if (enco_a>enco_a_initial+55 && enco_a<enco_a_initial+65)
        gripper_opened=1;
        break;
    else
        mymotora.Speed=6;
    end
end
mymotora.Speed=0;

%% main stuff is here

%% Adjusting for theta_1
move_motorc(set_point_c_pick,mymotorc,enco_reset_c,kp_c,ki_c,kd_c);
disp("c adjusted");

%% Adjusting for theta_2
move_motorb(set_point_b_pick,mymotorb,enco_top_b,kp_b,ki_b,kd_b);
disp("b adjusted");

%closing the gripper
for j=1:150
    mymotora.Speed=-9;
end
mymotora.Speed=0;
pause(0.5);
disp("gripper closed");
%% after pickup, lifting the arm to the mid position

move_motorb(halfway_setpoint,mymotorb,enco_top_b,kp_b,ki_b,kd_b);
disp("moving up");
end

function placing(mymotorb,mymotorc,mymotora,set_point_b_place,set_point_c_place,enco_top_b,enco_reset_c,enco_a_initial,kp_b,ki_b,kd_b,kp_c,ki_c,kd_c,halfway_setpoint)
%% drop locaiton main stuff is here
disp("Main stuff begins here");
move_motorc(set_point_c_place,mymotorc,enco_reset_c,kp_c,ki_c,kd_c);

%% taking the top as the calibration point
move_motorb(set_point_b_place,mymotorb,enco_top_b,kp_b,ki_b,kd_b);

%% opening the gripper
gripper_opened=0;
while (gripper_opened==0)
    enco_a=readRotation(mymotora);
    if (enco_a>enco_a_initial+55 && enco_a<enco_a_initial+65)
        gripper_opened=1;
        break;
    else
        mymotora.Speed=6;
    end
end

mymotora.Speed=0;
pause(0.5);

move_motorb(halfway_setpoint,mymotorb,enco_top_b,kp_b,ki_b,kd_b);
end

function[theta_1,theta_2]=para_known(z,polar_angle)
theta_2 = asind((z-77.175)/185)+45;
theta_1 = polar_angle;
end

function[theta_1,theta_2]= para_calc()

syms theta_1 theta_2;
z = input("Enter z value = ");
theta_2 = asind((z-77.175)/185)+45;
r= 185*cosd(theta_2-45)-67.175;
disp_r= ['Value of radius of the circle that the end effector can trace is ', num2str(r)];
disp(disp_r);

eq_xy = ['sqrt(x^2+y^2)=',num2str(r)];
correct_input=0;
correct_choice=0;
while(correct_choice==0)
    disp("Do you want to enter the coordinates of the ball in cartesian or polar coordinates? ");
    disp("1. Cartesian 2. Polar");
    choice = input("Enter your choice: ");
    switch choice
        case 1
            while (correct_input==0)
                disp("The value of x and y should be such that they satisfy the equation ");
                disp(eq_xy);
                
                x= input("Enter the value of x: ");
                y= input("Enter the value of y: ");
                r_input = sqrt(x^2+y^2);
                
                if (abs(r_input-r)<1)
                    correct_input= 1;
                    theta_1 = atan2d(y,x);
                    if(theta_1==180)
                        theta_1=-180;
                    end
                    correct_choice=1;
                else
                    disp("Please enter the value of x and y such that they follow the equation ");
                end
            end
            
            
        case 2
            while (correct_input==0)
                disp("Radius is the same as calculated. It should be betweeb 0 and -180 ")
                theta_1= input("Enter theta_1: ");
                if(theta_1<=0 && theta_1>=-180)
                    correct_input=1;
                    correct_choice=1;
                end
            end
            
        otherwise
            disp("Wrong Choice. ")
    end
end

destination_disp1 = ['theta_1 is ', num2str(theta_1),'.'];
destination_disp2 = ['theta_2 is ', num2str(theta_2),'.'];
disp("The destination angles are:  ");
disp(destination_disp1);
disp(destination_disp2);
%
end

function move_motorc(set_pointc,mymotorc,enco_reset_c,kp_c,ki_c,kd_c)


t_c=tic();
if set_pointc==0
    enco_c = readRotation(mymotorc)-enco_reset_c+10;
else
    enco_c = readRotation(mymotorc)-enco_reset_c;
end
prev_error_c = 0;
error_sum_c = 0;

error_c = set_pointc-enco_c;

t_c_current=toc(t_c);
dt_c=0;

z=1;
while (error_c<-2 || error_c>2)
    if(z==1)
        mymotorc.Speed = kp_c*error_c;
    else
        mymotorc.Speed = kp_c*error_c + ki_c*error_sum_c * dt_c +kd_c*(error_c-prev_error_c)/dt_c;
    end
    enco_c = readRotation(mymotorc) - enco_reset_c;
    prev_error_c = error_c;
    error_sum_c = error_sum_c + error_c;
    error_c = set_pointc-enco_c;
    dt_c = toc(t_c)-t_c_current-dt_c;
    z=z+1;
end
mymotorc.Speed = 0;
end

function move_motorb(set_pointb,mymotorb,enco_top_b,kp_b,ki_b,kd_b)

t_b=tic();
enco_b = readRotation(mymotorb)-enco_top_b;

prev_error_b = 0;
error_sum_b = 0;

error_b = set_pointb-enco_b;
t_b_current=toc(t_b);
dt_b=0;
z=1;
while (error_b<-1 || error_b>1)
    if(z==1)
        mymotorb.Speed = kp_b*error_b;
    else
        mymotorb.Speed = kp_b*error_b + ki_b*error_sum_b * dt_b +kd_b*(error_b-prev_error_b)/dt_b;
    end
    enco_b = readRotation(mymotorb)-enco_top_b;
    prev_error_b = error_b;
    error_sum_b = error_sum_b + error_b;
    error_b = set_pointb-enco_b;
    dt_b = toc(t_b)-t_b_current-dt_b;
    z=z+1;
end
mymotorb.Speed = 0;
end

