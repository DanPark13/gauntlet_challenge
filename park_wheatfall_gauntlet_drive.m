%%% Daniel Park % Elia's Wheatfall Gradient Drive Code
%% Initialize all the scaling variables for the drive
syms x y r phi d Xn Yn
clf
scale = .1;
wall_scale = 1;

%% Make the gradient equation for the obstacles and barrel of benevolence
eq = scale*-log(sqrt((x-1.41).^2 + (y+2).^2)) ...
    - scale*log(sqrt((x - 1).^2 + (y+.7).^2)) ...
    - scale*log(sqrt((x + .25).^2 + (y+1).^2)) ...
    + 3*log(sqrt((x -.75).^2 + (y+2.5).^2));

%% Plot the walls
% Instance of the wall
wall = 0.3;

% x-axis walls on y = -1.5, 2.5
for a = -1.5:wall:2.5
    eq = eq - wall_scale*log(sqrt((x-a).^2 + (y+3.37).^2)) - wall_scale*log(sqrt((x-a).^2 + (y-1).^2));
end

% y-axis walls on y = -3.37, 1
for b = -3.37:wall:1
    eq = eq - wall_scale* log(sqrt((x+1.5).^2 + (y-b).^2)) - wall_scale*log(sqrt((x-2.5).^2 + (y-b).^2));
end

%% Draw the Neato path driven using the equation

%Starting parameters
start_position=[0,0];
base_equation = eq;
lamda  =  0.2;
delta = 0.9;
r = 0.235/2; %Radius between two wheels
speed = .2;

%Store gradient equation and second position
gradient_equation = [-1.*diff(base_equation,x);-1.*diff(base_equation,y)];
next_position = (lamda .* double(subs(gradient_equation,[x,y],[start_position])))' + start_position;
start_angle = deg2rad(350);
%Create array of position values
positions = [start_position];
angles = [start_angle];

% Get all the Neato positions over time
while double(norm(lamda.* subs(gradient_equation,[x,y],[next_position]))') >.1
    %Append positions to a list
    positions = [positions;next_position];
    
    %Calculate angle transitions
    next_angle = atan2(abs(double(subs(gradient_equation(2,:),[x,y],[next_position]))),abs(double(subs(gradient_equation(1,:),[x,y],[next_position]))));
    angles = [angles;next_angle];
    
    %Calculate next position
    next_position = (lamda .* double(subs(gradient_equation,[x,y],[next_position])))' + next_position;
    lamda  = lamda * delta;
end

%% Plot the Neato path onto a graph
hold on
xlabel("x(m)")
ylabel("y(m)")
title("Gauntlet Challenge Neato Travel Path")
plot(positions(:,1),positions(:,2))
ylim([-3 inf])
plot(1,-.7,".",'MarkerSize', 30)
plot(-.25,-1,".",'MarkerSize', 30)
plot(1.41,-2,".",'MarkerSize', 30)
plot(.75,-2.5,".",'MarkerSize', 30)
legend("Travel Path","Obstacle 1","Obstacle 2","Obstacle 3","Barrel of Benevolence",'Location','best')
axis equal
positions
angles = deg2rad(360) - angles;
rad2deg(angles)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Create the Lidar Scans from various positions on the Gauntlet
sub = rossubscriber('/scan');

hold off
% Position #1
placeNeato(0,0,1,0)
pause(2);
scan_message = receive(sub);
r_1 = scan_message.Ranges(1:end-1);
theta_1 = deg2rad([0:359]')
data_1 =[r_1,theta_1]

% Position #3
placeNeato(1.5,-2.5,1,0)
pause(2);
scan_message = receive(sub);
r_3 = scan_message.Ranges(1:end-1);
theta_3 = deg2rad([0:359]');

% Position #4
placeNeato(.5,-1.7, 1,0)
pause(2);
scan_message = receive(sub);
r_4 = scan_message.Ranges(1:end-1);
theta_4 = deg2rad([0:359]');
map = []
%Combine values
r_all = [r_1 r_3 r_4];
Xn =0;
Yn =0;

%% Plot the Lidar Figure
clf
r1 = r_all(:,1);
r3 = r_all(:,2);
r4 = r_all(:,3);

hold off
theta = deg2rad([0:359]');
d= .084;
phi = 0;
%Plot 1
data_1 = [];
for i=1:359
    Xn = 0;
    Yn = 0;
    data_1 =[data_1 , [r1(i)*cos(theta(i) + phi)-d*cos(phi) + Xn;r1(i)*sin(theta(i)+phi)-d*sin(phi)+Yn]];
end

%Plot 3
data_3 = [];
for i=1:359
    Xn = 1.5;
    Yn = -2.5;
    data_3 =[data_3 , [r3(i)*cos(theta(i) + phi)-d*cos(phi) + Xn;r3(i)*sin(theta(i)+phi)-d*sin(phi)+Yn]];
end

%Plot 4
data_4 = [];
for i=1:359
    Xn = 0.5;
    Yn = -1.7;
    data_4 =[data_4 , [r4(i)*cos(theta(i) + phi)-d*cos(phi) + Xn;r4(i)*sin(theta(i)+phi)-d*sin(phi)+Yn]];
end

% Label the LIDAR plot
hold on;
title("Gauntlet Complete layout")
xlabel("(m)")
ylabel("(m)")
plot(data_1(1,:),data_1(2,:),"o")
axis equal
plot(data_3(1,:),data_3(2,:),"o")
axis equal
plot(data_4(1,:),data_4(2,:),"o")
axis equal

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Drive Neato

%This line says we are going to publish to the topic `/raw_vel' 
%which are the left and right wheel velocities
pubvel = rospublisher('/raw_vel');

%Here we are creating a ROS message
message = rosmessage(pubvel);  

% Send the velocity commands to the NEATO
send(pubvel, message);

% Place the Neato at the starting position
placeNeato(start_position(:,1),start_position(:,2),1,0,1);
pause(3)

% Drive Neato along the path earlier found before
for n = 1 : length(positions(:,1)) - 1
    % Preliminary variables for turning
    angularSpeed = .2;
    turnAngle =angles(n+1) - angles(n)
    turn_sign = sign(turnAngle)
    
    % Calculates how long in seconds to turn for
    turnTime = double(turnAngle) / angularSpeed;

    % Print a message for how much the Neato turned (for debugging)
    message.Data = [-turn_sign*angularSpeed*r,turn_sign*angularSpeed*r]
    send(pubvel, message);

    % Turn the Neato
    startTurn = rostic;
    while rostoc(startTurn) < turnTime
        pause(0.01);
    end
    
    % Drive forward for a specific known distance
    driveforward(norm(positions(n+1,:)-positions(n,:)),speed);
    % Stop before repeating the steps to find the turn before moving forward
    pause(.1)
end