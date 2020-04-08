 
%% Given constants
clear
clc
g = 9.8;    %accn due to gravity
m = 1;      %weight of the payload
A = 0.015;  %cross sectional area of payload
rho = 1.2;  %air density
dt = 0.01;  %freefall prediction time stamp
cd = 1.7;   %drag coefficent

z_0 = 0.1;  %surface roughness, depends upon where payload is dropped

%below defined constant deals with the time lag
time_loop = 0.08; %time lag in loop
time_payload_mechanism = 2; %time to open payload mechanism

time_delay = time_loop + time_payload_mechanism;

a = 1; % this changes to '-1' when difference in angle between theta and phi exceeds pi
%NORMAL CASE
%whenever theta > phi then the resultant vector of direction 'dirn' is below
%the UAV velocity vector ( below means '-' in 1st and 2nd quadrant),
%BUT 
%when the difference between theta and phi is greater than 180 degree then the
%resultant vector of direction 'dirn' is still below the UAV velocity
%trajectory but is in 3rd and 4th quadrant ( and here below means ' + '). So
%to change this direction of + to - and - to + 'a' is used.


%% Given variables from sensor
wind_speed_north = 10; 
wind_speed_east = -10;
airspeedUAV = 18;
global_altitude = 188.5;
target_altitude = 5.5;
% Initial calculation from these variables from sensors
ground_distance = global_altitude - target_altitude;

%calculate the relative direction of the wind
wind_speed_normalized = sqrt(wind_speed_north ^ 2 + wind_speed_east ^ 2);

%this gives the direction the plane is flying
current_n = 0.0;
current_e = 0.0;
target_n = 100.0;
target_e = -100.0;

%the angle in which airplane is going is theta
theta = wrapToPi(atan2(target_e - current_e,target_n - current_n));
%the angle in which wind is blowing is phi
phi = wrapToPi(atan2(wind_speed_east,wind_speed_north));

%change theta and phi into [0 , 2*pi] 
if(theta < 0)
    theta = 2 * pi + theta;
end
if(phi < 0)
    phi = 2 * pi + phi;
end

%direction of wind wrt direction of plane
% although C gives the direction of wind wrt plane, it doesn't give the
% direction in which the payload will drop. To find that we have to use law
% of cosines

%below statement makes difference angle positive
if phi > theta
    C = (phi - theta);
elseif phi < theta
    C = theta - phi;
end
%calculate ground speed of UAV
%the wind direction + is taken along the UAV direction
ground_speed_body = airspeedUAV + wind_speed_normalized * cos(C);

% if wind direction and plane direction isn't the same, calculate 'dirn' -
% which is direction in which payload will drop.
if(theta ~= phi)
    if(C < pi)
        C = pi - C;
        a = 1;
    else %whenever the difference of angle between theta and phi exceeds 180 degrees
        C = C - pi;
        a = -1;
    end
    relative = sqrt(airspeedUAV ^ 2 + wind_speed_normalized ^ 2 - 2 * airspeedUAV * wind_speed_normalized * cos(C));
    dirn = (acos((relative ^ 2 + airspeedUAV ^ 2 - wind_speed_normalized ^ 2)/(2*relative * airspeedUAV)));
%if wind direction and plane direction is same then, direction in which
%payload will drop is equal to direction in which plane is flying
else
    dirn = theta;
end

%%
% Initialize all working variables
vz = 0;     %velocity in z direction
az = 0;     %acceleration in z direction
z  = 0;     %intermediate height calculated
h = 0;      %height over target
vw = 0;     %windspeed
vx = 0;     %velocity in x direction
ax = 0;     %acceleration in x direction
x = 0;      %displacement in NE plane in NED frame
vrx  = 0;   %velocity in x direction with relative to wind

v = 0;      %relative speed vector
fd = 0;     %drag force
fdx = 0;    %drag force in x direction
fdz = 0;    %drag force in z direction

wind_direction_north = 0;   %vector of wind speed north
wind_direction_east = 0;    %vector of wind speed east

% to calculate point of drop in NED frame
% N = x, E = y
x_d = 0;
y_d = 0;

% assign initial value
h = ground_distance;
az = g;
ax = 0; %on the assumption that drone isn't accelerating
vx = ground_speed_body; %confused whether to take airspeed or ground speed
vz = 0; %initial velocity in z direction of payload = 0

% compute distance
while h > 0.01
%calculation along z direction in NED frame
vz = vz + az * dt;
z = z + vz * dt ;
h = h - z;

%calculation of wind velocity according to variation in height
vw = wind_speed_normalized * (log(h / z_0) / log(ground_distance / z_0));
%%%%%%%%%%%%%%%comment: windspeed very high from calculation %%%%%%%%%

%calculation along x direction i.e NE plane in NED frame
vx = vx + ax * dt;
x = x + vx * dt;
vrx = vx + vw;

%calculation of drag force
v = sqrt(vz ^2 + vrx ^ 2);
fd = 0.5 * rho * cd * A * (v^2);
fdx = fd * vrx / v;
fdz = fd * vz / v;

%calculation of acceleration
az = g - fdz / m;
ax = -fdx / m;
%plot(x,h,'co');
%hold on;
%axis tight

end

% add time delay to x vector
x = ground_speed_body * time_delay + x;

%
%below are three cases

if(phi == theta)%if the relative payload path is same as plane path
    disp('dirn == theta');
    RP_n = target_n - x*cos(dirn);
    RP_e = target_e - x*sin(dirn);
elseif(theta < phi) %if the relative payload path is above the plane path
    disp('theta < phi');
    RP_n = target_n - x * cos(theta + a * dirn);
    RP_e = target_e - x * sin(theta + a * dirn);
elseif (theta > phi || phi == 0)% if the relative payload path is below the plane path or no wind is blowing
    disp('theta > phi');
    RP_n = target_n - x * cos(theta - a * dirn);
    RP_e = target_e - x * sin(theta - a * dirn);
end
plot([current_n target_n], [current_e target_e])
hold on 

plot(target_n,target_e,'ro');
hold on;
plot(RP_n,RP_e,'ko');

axis([-200,200,-200,200])







