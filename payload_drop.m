
%% Given constants
clear
g = 9.8;    %accn due to gravity
m = 1;      %weight of the payload
A = 0.015;  %cross sectional area of payload
rho = 1.2;  %air density
dt = 0.01;  %freefall prediction time stamp
cd = 0.1;   %drag coefficent

z_0 = 0.1;  %surface roughness, depends upon where payload is dropped

%below defined constant deals with the time lag
time_loop = 0.08; %time lag in loop
time_payload_mechanism = 2; %time to open payload mechanism

time_delay = time_loop + time_payload_mechanism;


%% Given variables from sensor
wind_speed_north = 5; 
wind_speed_east = 5;
velocity_north = 30.0;
velocity_east = 1.0;
global_altitude = 188.5;
target_altitude = 5.5;
% Initial calculation from these variables from sensors
ground_distance = global_altitude - target_altitude;
ground_speed_body = sqrt(velocity_north ^ 2 + velocity_east ^ 2);
wind_speed_normalized = sqrt(wind_speed_north ^ 2 + wind_speed_east ^ 2);


%% Initialize all working variables
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

%% compute distance
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
plot(x,h,'co');
hold on;
axis tight
end

%% add time delay to x vector
x = ground_speed_body * time_delay + x;


if wind_speed_normalized < 0.5  %if there is no wind, an arbitraril direction is chosen
    wind_direction_north = 1.0;
    wind_direction_east = 0.0;
else     
    wind_direction_north = wind_speed_north / wind_speed_normalized;
    wind_direction_east = wind_speed_east / wind_speed_normalized;
end
%%
%variable below defines (x_r,y_r) = (0,0) i.e origin
%It is used to findout coordinates of target, payload drop in reference to origin 
x_r = 0;
y_r = 0;    

% drop point calculate with release point xr,yr
x_d = x_r + x * wind_direction_north;
y_d = y_r + x * wind_direction_east;

plot(x_d,y_d,'ro');
hold on;
plot(x_r,y_r,'ko');

%% release point calculate with drop point xd,yd
x_d = 20;
y_d = 15;
x_r = x_d - x * wind_direction_north;
y_r = y_d - x * wind_direction_east;

plot(x_d,y_d,'ro');
hold on;
plot(x_r,y_r,'ko');







