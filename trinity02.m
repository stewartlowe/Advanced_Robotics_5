function [computation_cycles,total_travel_dist,goal_reached,num_crashes,wall_crashes] = trinity02(sensor_noise,movement_noise,plotting,movie)

% This function simulates a mobile robot driving around the Trinity College
% Home Robot Fire-fighting course
% Distance measurements are achieved by either Sharp IR or SRF02 ultrasonic
% range finders
% The robot is propelled by differential drive provided by two Parallax
% continuous-rotation servo motors
%
% The default arena is the Trinity Fire-Fighting Robot Competition map with
% a generic differential-drive robot:
%
%              -------
%            /    O    \ 
%         X |    UVU    | X
%         X=|    > <    |=X
%         X |     ^     | X
%            \    O    / 
%              -------
%
% legend:
%          V   front range finder
%         > <  side range finders
%          ^   rear range finder
%          O   line sensor
%          U   flame sensor
%         | -  chassis edges
%          X   wheel
%          =   axle
%
% specs:
%
%    wheelbase     20 cm
%    length        20 cm
%    range dist    80 cm
%    range angle   15 deg
%    flame dist    60 cm
%    flame angle    5 deg
%    max speed    ~17 cm/s (50rpm motors with 6.65cm dia. tyres)


% INPUT PARAMETERS
% 'movie' value defines whether an output movie file is created
%         0 = no movie file, on-screen simulation only
%         1 = create a movie file in addition to the on-screen simulation
if nargin<4
  movie = 0;  % if the movie flag is not set, default to 'no movie file'
end
%If plotting is set to 1, the simulation will be displayed on screen,
%if set to 0 the simulation will run headlessly
%Plotting set to 1 by default and overridden to 1 if movie is enabled
if nargin<3 || movie==1
  plotting = 1;
end
% 'movement_noise' is a varable to hold the amount of Gaussian movement noise
% as a percentage of the maximum speed
if nargin<2
  movement_noise = 0.0;  % if the movement_noise is not defined, default to 'no movement noise'
end
% 'sensor_noise' is a varable to hold the amount of Gaussian sensor noise
% as a percentage of the maximum sensor range
if nargin<1
  sensor_noise = 0.0;  % if the sensor_noise is not defined, default to 'no esnsor noise'
end



% INITIALISATION

% read the default Trinity arena map (from the 2004 competition)into the array 'arena_map'
% each map location represents a 2 x 2cm square in a 2.4 x 2.4m arena
max_grid = 122;  % the arena size is 244 x 244cm / 2 = 122 x 122 grid elements
arena_map = '2004';
fid = fopen(['trinity_course_' arena_map '.dat']);
for r=max_grid:-1:1
  arena_line = fgetl(fid);  % read a line of the arena map file
  for c=1:max_grid
    arena_str(r,c) = arena_line(c);  % assign each character of the current arena map line to a cell in the 'arena_str' matrix
  end
end
fclose(fid);

% seed the random number generator for sensor and movement noise
rng('shuffle');  % use a non-fixed value for a more random set of values, e.g. rng('shuffle');

% define the typical hallway width in the arena, which is 46cm / 2  (for scale)
hallway_width = 46/2;  % cm

% 'vehicle_pos' defines the position and heading of the vehicle
%         [x position, y position, Cartesian heading];
%         . [0,0,h] is the lower-left corner of the arena
%         . x values increase to the right; y values increase to the top
%         . the vehicle is 3x3 in size with differential drive about the midpoint
%         . this initial definition represents the starting position and heading
%         . default = [110,59,90]
[r,c] = find(arena_str=='O');
arena_str(arena_str=='O') = '0';
vehicle_pos = [c,r,90];  % 'home' position facing north
%vehicle_pos = [110, 59, 90];  % 'home' position facing north
%vehicle_pos = [ 16,105,130];  % 'goal reached' test position facing candle 'a'
%vehicle_pos = [ 82, 87, -45];  % 'goal reached' test position facing candle 'd'
%vehicle_pos = [ 80, 86, -40];  % 'goal reached' test position facing candle 'd'


% choose the location of the candle (goal)
% it can be positioned at one of the 10 room locations, indicated by a letter
% . if found, change the letter to '2' for a 'goal' object and the
%   associated surroundling line (upper-case same letter) to '1' for a 'line' object
% . otherwise, change the letter to '0' and also any associated line objects to '0'
%goal_val = double('a') + round(rand*9,0);  % random goal position - ASCII value of the goal position letter
goal_val = double('b');  % choose a fixed goal position for debugging purposes, position 'b'
for g = double('a'):double('j')
  if (goal_val==g)
    arena_str(arena_str==char(g)) = '2';
    arena_str(arena_str==char(g-32)) = '1';
  else
    arena_str(arena_str==char(g)) = '0';
    arena_str(arena_str==char(g-32)) = '0';
  end
end

% convert the string-based arena map values into numeric values
arena = zeros(max_grid);
for r=max_grid:-1:1
  arena_line = arena_str(r,:);
  for c=1:max_grid
    arena(r,c) = str2num(arena_line(c));  %#ok<ST2NM>
  end
end


% 'wheelbase' is the distance between the differential drive wheels
%         . the midpoint between the wheels is also considered the 'centre' of the robot
%         . default = 10  (20cm * 1/2 scale of the arena map)
wheelbase = 10;

% 'max_speed' is the maximum speed that a wheel can drive in cm/s / 2
%         . our tyres are 66.5mm in diameter and the drive motors turn at 50rpm
%         . tyre circumference = PI * d = PI * 6.65cm
%         . rps = 50rpm / 60sec/min
max_speed = pi*6.65*50/60 / 2;  % cm/s
  
% vehicle_size defines the outer extremes of the vehicle
%         for collision purposes, the vehicle is considered round, with
%         radius = centre to most extreme point / 2
%         (i.e. our robot is a ciruclar platform with a radius of 10cm = 5)
vehicle_size = 5;

% 'sensors' defines the type, position, and characteristics of each sensor on the vehicle
%         sensors = [sensor type, sensor range, sensor field-of=view, orientation, x pos, y pos]
%         . sensor type
%           1 = line detection sensor
%           2 = goal seeking sensor for target detection
%           3 = range-finding sensor for obstacle avoidance
%         . sensor range
%           maximum sensing distance in cm/2
%         . sensor field-of-view
%           total field-of-view in degrees (e.g. +/-45deg = 90)
%         . orientation
%           mounting angle for the midpoint of the field-of-view as
%           referenced with the vehicle pointed to the right at 0deg
%           (e.g. a sensor mounted 15deg anti-clockwise from centreline would be 15)
%         . x pos and y pos are the x- and y-coordinates of the sensor
%           location, relative to the midpoint of the drive axles in cm/2
%           foreward = +x, rearward = -x, left side = +y, right side = -y

% create some constants that will be helpful for the control action
LD_FRONT = 1;
LD_REAR  = 2;
FL_LEFT  = 3;
FL_RIGHT = 4;
IR_FRONT = 5;
IR_REAR  = 6;
IR_LEFT  = 7;
IR_RIGHT = 8;
%                     typ rng  FoV   ang     x      y
sensors(LD_FRONT,:) = [1,   2, 360,    0,   4.0,   0.0];  % line detector
sensors(LD_REAR, :) = [1,   2, 360,    0,  -4.0,   0.0];  % line detector
sensors(FL_LEFT, :) = [2,  30,   5,   -5,   1.5,   1.5];  % flame sensor
sensors(FL_RIGHT,:) = [2,  30,   5,    5,   1.5,  -1.5];  % flame sensor
sensors(IR_FRONT,:) = [3,  40,  15,    0,   1.5,   0.0];  % IR range finder (fore)
sensors(IR_REAR, :) = [3,  40,  15,  180,  -1.5,   0.0];  % IR range finder (aft)
sensors(IR_LEFT, :) = [3,  40,  15,   90,   0.0,   1.5];  % IR range finder (left)
sensors(IR_RIGHT,:) = [3,  40,  15,  -90,   0.0,  -1.5];  % IR range finder (right)

% note the number of sensors
% used to check each sensor against detectable objects
num_sensors = size(sensors,1);

% 'sensor_hist' is an array of sensor distance values for the present and
% past two computation cycles
sensor_hist = zeros(4,num_sensors);

% 'vehicle_path' is a variable that holds the robot position at each
% calculation cycle
vehicle_path(1,:) = vehicle_pos(1:2);

% 'odometry' is a variable to hold the perceived distances travelled by
% each wheel, which may or may not reflect how far the robot actually
% travels
odometry = zeros(1,2);

% 'total_travel_dist' is a variable to hold to total distance travelled by
% the vehicle; this represents the actual distance travelled by the robot
total_travel_dist = 0.0;

% 'goal_reached' is a variable that indicates whether the goal was located
% or not
goal_reached = 0;
% 'num_crashes' is a variable to hold the number of steps where an obstacle
% crash occurs
num_crashes = 0;
% 'wall_crashes' is a variable to indicate when the vehicle left the arena
wall_crashes = 0;

% dt is the time increment between simulation renders
%         . sensor values and vehicle movement are calculated at each interval (e.g. 250ms)
dt = 0.250;


if movie==1  % if the movie flag is set, create the output movie file
  % setup video output file to record the simulation
  vid  = VideoWriter(['./trinity02_c' arena_map '_sn' sprintf('%5.3f',sensor_noise) '_mn' sprintf('%5.3f',movement_noise) '_(' num2str(vehicle_pos(1)) ',' num2str(vehicle_pos(2)) ',' num2str(vehicle_pos(3)) ').mp4'],'MPEG-4');
  open(vid);
  dup_frames = 7;          % number of figure frames to duplicate, to slow the video rate and be relatively close to real-time
end

% 'control_step' is a variable to progress through the various stages of
% reaching the first room
control_step = 1;

% SIMULATION LOOP
%
% loop through the simulation calculations for a fixed number of times
%         . default = 25
% some guidelines for chosen courses:
%         trinity_course01
%         .  27 roughly searches 1 room
%         .  66 roughly searches 2 rooms
%         . ??? roughly searches 3 rooms
%         . ??? roughly searches 4 rooms

for t = 0:dt:100

  % record ho many computation cycles the vehicle uses to reach the goal,
  % exit the arena, or complete the number of computation loops in the
  % t=0:dt:xxx FOR LOOP
  computation_cycles = t/dt + 1;

  
  
  % FIND THE NEAREST OBJECT FOR EACH SENSOR
  
  % calculate the positions of the sensors, in world coordinates, for
  % object detection distances
  % start with the natural x- and y-coordinates of each sensor
  sensor_pos = sensors(:,5:6);
  % then perform a rotation transformation based on the robot heading
  transform_angle = vehicle_pos(3)*pi/180;
  for i=1:num_sensors
    sensor_pos(i,:) = ([[cos(transform_angle) -sin(transform_angle)]
                        [sin(transform_angle)  cos(transform_angle)]] * sensor_pos(i,:)')';
  end
  sensor_pos = sensor_pos + vehicle_pos(1:2);
  
  % for displaying the current vehicle status in the status bar,
  % re-calculate the sensor positions relative to the vehicle oriented to
  % 90deg (facing 'north')
  sensor_plot = sensors(:,5:6);
  % then perform a rotation transformation based on the robot facing 'north'
  transform_angle = 90*pi/180;
  for i=1:num_sensors
    sensor_plot(i,:) = ([[cos(transform_angle) -sin(transform_angle)]
                         [sin(transform_angle)  cos(transform_angle)]] * sensor_plot(i,:)')';
  end

  % initialise the sensor_dist array to see no objects (max range + 1)
  sensor_dist = sensors(:,2)+1;
  
  % check each sensor for detectable objects within range
  for i=1:num_sensors
    
    % only search the grid locations within range of the sensor
    %   cr - check row
    %   cc - check column
    cr_min = floor(sensor_pos(i,2)-sensors(i,2));
    cr_max = ceil(sensor_pos(i,2)+sensors(i,2));
    cc_min = floor(sensor_pos(i,1)-sensors(i,2));
    cc_max = ceil(sensor_pos(i,1)+sensors(i,2));
    if cr_min<1;             cr_min=1;   end             % if the search cell index is beyond the arena
    if cr_max>max_grid; cr_max=max_grid; end             % wall, set the cell value to be equal to a wall cell
    if cc_min<1;             cc_min=1;   end
    if cc_max>max_grid; cc_max=max_grid; end
  
    for cr=cr_min:cr_max
      for cc=cc_min:cc_max
      
        % find nearest detectable objects
        %arena(cr,cc)  % used for debugging
        %sensors(i,1)  % used for debugging
        if arena(cr,cc)==sensors(i,1)  % the arena marker is the same as the sensor type (e.g. 1 = line; 2 = goal; 3 = obstacle)
          if sensors(i,3)<360  % for limited FoV sensors
            % calculate the angle between the current obstacle/goal and the current sensor
            object_angle = atan2(cr-sensor_pos(i,2),cc-sensor_pos(i,1));
            % target_angle_deg = target_angle *180/pi;  % used to debug obstacle and sensor angles
            sensor_heading1 = vehicle_pos(3)+sensors(i,4)+sensors(i,3)/2;
            sensor_heading2 = vehicle_pos(3)+sensors(i,4)-sensors(i,3)/2;
            if sensor_heading1>object_angle*180/pi+180; sensor_heading1=sensor_heading1-360; end  % set the sensor heading values
            if sensor_heading1<object_angle*180/pi-180; sensor_heading1=sensor_heading1+360; end  % to be close to the obstacle angle
            if sensor_heading2>object_angle*180/pi+180; sensor_heading2=sensor_heading2-360; end  % values to ensure better numerical
            if sensor_heading2<object_angle*180/pi-180; sensor_heading2=sensor_heading2+360; end  % comparisons of obstacles within FoV

            % object_angle_d   = object_angle*180/pi;     % used to debug the sensor angle evaluations
            % sensor_heading1r = sensor_heading1*pi/180;  % used to debug the sensor angle evaluations
            % sensor_heading2r = sensor_heading2*pi/180;  % used to debug the sensor angle evaluations
            if object_angle<=sensor_heading1*pi/180 && object_angle>=sensor_heading2*pi/180
              object_dist = sqrt((cc-sensor_pos(i,1))^2+(cr-sensor_pos(i,2))^2) + (rand()-0.5)*sensor_noise*sensors(i,2);
              if object_dist<sensors(i,2) && object_dist < sensor_dist(i)
                sensor_dist(i) = object_dist;
                % fprintf('cr: %i  cc: %i    sr: %i  sc: %i   dist: %8.4f\n',cr,cc,sr,sc,object_dist);  % used for debugging the sensor distance calculations
              end
            end
          else % for omni-directional sensors
            object_dist = sqrt((cc-sensor_pos(i,1))^2+(cr-sensor_pos(i,2))^2);
            if object_dist<=sensors(i,2) && object_dist < sensor_dist(i)
              sensor_dist(i) = object_dist;
            end
          end
        end  % end of 'arena(cr,cc)==sensors(i,1)' IF CONDITION
          
      end  % end of 'cc' FOR LOOP
     end  % end of 'cr' FOR LOOP
  end  % end of 'i=1:num_sensors' FOR LOOP

  % if the front range finder detects an obstacle at a closer distance than
  % the flame sensors, it is likely that a wall is between the goal and the
  % flame sensors
  % in this case, clear any detected goal distances
  if sensor_dist(IR_FRONT) < sensor_dist(FL_LEFT)
    sensor_dist(FL_LEFT)  = sensors(FL_LEFT,2) +1;
  end
  if sensor_dist(IR_FRONT) < sensor_dist(FL_RIGHT)
    sensor_dist(FL_RIGHT) = sensors(FL_RIGHT,2)+1;
  end
  
  %sensor_dist  % used to debug the final sensor distance results
   
% use a series of shifting operations to rotate each set of sensor data to the next older index in
% the sensor_hist array, then store the current sensor data in the first index position
  sensor_hist(4,:) = sensor_hist(3,:);
  sensor_hist(3,:) = sensor_hist(2,:);
  sensor_hist(2,:) = sensor_hist(1,:);
  sensor_hist(1,:) = sensor_dist;  
  
  %PLOTTING
  if plotting==1

    fig = figure(1);

    clf;  % clear the figure in prepmaration for drawing the new arena state
    
    % plot the arena
    hold on
    for r=1:max_grid
      for c=1:max_grid
        if arena(r,c)==3
          plot(c,r,'or')  % plot an obstacle as a red circle
        end
        if arena(r,c)==2
          plot(c,r,'sg')  % plot a goal as a green square
        end
        if arena(r,c)==1
          plot(c,r,'sk')  % plot white lines as black squares
        end
      end
    end
    axis equal
    axis([0 max_grid+41 0 max_grid+1])

    % plot the vehicle as a circle with radius 'vehicle_size'
    circumference = 0:36:360;
    vehicle_plot = [vehicle_size*cos(circumference*pi/180)+vehicle_pos(1);
                    vehicle_size*sin(circumference*pi/180)+vehicle_pos(2)];
    plot(vehicle_plot(1,:),vehicle_plot(2,:),'Color','k')
    
    % plot the vehicle outline in the status bar
    status_vehicle_pos = [max_grid+21 max_grid-30];
    text(status_vehicle_pos(1)-15,status_vehicle_pos(2)+25,'VEHICLE STATUS','FontSize',12,'Color','r');
    vehicle_plot = vehicle_plot - vehicle_pos(1:2)' + status_vehicle_pos';
    plot(vehicle_plot(1,:),vehicle_plot(2,:),'Color','k')
    
    % plot sensor locations
    sensor_color = ['k' 'g' 'r'];  % line detector = black; goal-seeking = green; obstacle-avoidance = yellow
    for i=1:num_sensors
      plot(sensor_pos(i,1),sensor_pos(i,2),'+','Color',sensor_color(sensors(i,1)))
      % sensor viewing areas
      sensor_x = zeros(12,1);
      sensor_y = zeros(12,1);
      sensor_x(1) = sensor_pos(i,1);
      sensor_y(1) = sensor_pos(i,2);
      s=2;
      for angle = sensors(i,3)/2:-sensors(i,3)/10:-sensors(i,3)/2
        sensor_x(s) = sensor_pos(i,1)+sensors(i,2)*cos((vehicle_pos(3)+sensors(i,4)+angle)*pi/180);
        sensor_y(s) = sensor_pos(i,2)+sensors(i,2)*sin((vehicle_pos(3)+sensors(i,4)+angle)*pi/180);
        s=s+1;
      end
      sensor_x(s) = sensor_x(1);
      sensor_y(s) = sensor_y(1);
      plot(sensor_x,sensor_y,'LineStyle','--','Color',sensor_color(sensors(i,1)))
      
      % plot sensor positions on the status bar vehicle
      sensor_plot(i,:) =  sensor_plot(i,:) + status_vehicle_pos;
      plot(sensor_plot(i,1),sensor_plot(i,2),'+','Color',sensor_color(sensors(i,1)))
    end
    
    % plot the vehicle path
    plot(vehicle_path(:,1), vehicle_path(:,2),'--b')

    % plot the control step value
    text(118,120,sprintf('%2i',control_step),'FontSize',10,'Color','r');
    
    % populate the status bar with current vehicle values
    % sensor readings:
    text(status_vehicle_pos(1)+ 4,status_vehicle_pos(2)+15,sprintf('%5.2f',sensor_dist(LD_FRONT)),'HorizontalAlignment','right','FontSize',10,'Color',sensor_color(sensors(LD_FRONT,1)));
    text(status_vehicle_pos(1)+ 4,status_vehicle_pos(2)-15,sprintf('%5.2f',sensor_dist(LD_REAR)) ,'HorizontalAlignment','right','FontSize',10,'Color',sensor_color(sensors(LD_REAR ,1)));
    text(status_vehicle_pos(1)- 8,status_vehicle_pos(2)+10,sprintf('%5.2f',sensor_dist(FL_LEFT)) ,'HorizontalAlignment','right','FontSize',10,'Color',sensor_color(sensors(FL_LEFT ,1)));
    text(status_vehicle_pos(1)+16,status_vehicle_pos(2)+10,sprintf('%5.2f',sensor_dist(FL_RIGHT)),'HorizontalAlignment','right','FontSize',10,'Color',sensor_color(sensors(FL_RIGHT,1)));
    text(status_vehicle_pos(1)+ 4,status_vehicle_pos(2)+10,sprintf('%5.2f',sensor_dist(IR_FRONT)),'HorizontalAlignment','right','FontSize',10,'Color',sensor_color(sensors(IR_FRONT,1)));
    text(status_vehicle_pos(1)+ 4,status_vehicle_pos(2)-10,sprintf('%5.2f',sensor_dist(IR_REAR)) ,'HorizontalAlignment','right','FontSize',10,'Color',sensor_color(sensors(IR_REAR ,1)));
    text(status_vehicle_pos(1)- 8,status_vehicle_pos(2)   ,sprintf('%5.2f',sensor_dist(IR_LEFT)) ,'HorizontalAlignment','right','FontSize',10,'Color',sensor_color(sensors(IR_LEFT ,1)));
    text(status_vehicle_pos(1)+16,status_vehicle_pos(2)   ,sprintf('%5.2f',sensor_dist(IR_RIGHT)),'HorizontalAlignment','right','FontSize',10,'Color',sensor_color(sensors(IR_RIGHT,1)));
    % odometry:
    text(status_vehicle_pos(1)- 8,status_vehicle_pos(2)- 5,sprintf('%6.2f',odometry(1))          ,'HorizontalAlignment','right','FontSize',10,'Color','b');
    text(status_vehicle_pos(1)+16,status_vehicle_pos(2)- 5,sprintf('%6.2f',odometry(2))          ,'HorizontalAlignment','right','FontSize',10,'Color','b');
    % vehicle heading
    text(status_vehicle_pos(1)-11,status_vehicle_pos(2)-25,sprintf('heading: %+7.2f',vehicle_pos(3)),'FontSize',10,'Color','k');
    % computation cycles
    text(status_vehicle_pos(1)-16,status_vehicle_pos(2)-35,sprintf('computation cycles: %4i',computation_cycles),'FontSize',10,'Color','k');
    % obstacle collisions
    text(status_vehicle_pos(1)-15,status_vehicle_pos(2)-45,sprintf('obstacle collisions: %2i',num_crashes),'FontSize',10,'Color','r');
    
    hold off
    drawnow;
  end

    % check if the vehicle has gone beyond the arena boundaries
    %}
    if vehicle_pos(1) < 1+vehicle_size || vehicle_pos(1) > max_grid-vehicle_size || vehicle_pos(2) < 1+vehicle_size || vehicle_pos(2) > max_grid-vehicle_size
      wall_crashes = 1;
      if plotting==1
        text(31,max_grid/2,'WALL CRASH!','FontSize',40,'Color','r');
        drawnow;
      end
      break
    end
    % check if the vehicle has collided with an obtacle
    for r=floor(vehicle_pos(2)-vehicle_size):ceil(vehicle_pos(2)+vehicle_size)
      for c=floor(vehicle_pos(1)-vehicle_size):ceil(vehicle_pos(1)+vehicle_size)
        if arena(r,c)==3
          num_crashes = num_crashes + 1;
          if plotting==1
            text(15,max_grid/2,'OBSTACLE CRASH!','FontSize',40,'Color','r');
            drawnow;
          end
          break
        end
      end
    end
    % check if the goal has been reached, but only for modes with
    % goal-seeking sensors active
    %}
    if sensor_dist(LD_FRONT)<sensors(LD_FRONT,2) && sensor_dist(FL_LEFT)<sensors(FL_LEFT,2) && sensor_dist(FL_RIGHT)<sensors(FL_RIGHT,2)
      goal_reached = 1;
      if plotting==1
        text(27,max_grid/2,'GOAL REACHED!','FontSize',40,'Color','r');
      end
      break;
    end
    if plotting==1
      drawnow; % force the plot to be drawn, depsite the tight calculation loop
    end
  
  if movie==1  % if the movie flag is set, add figure frames to the movie file
    % save multiple copies of the current plot to the video file to
    % slow down the playback
    for i=1:dup_frames
      img = getframe(fig);
      writeVideo(vid,img);
    end
  end
  %}

  
  
 
  
  
  % CONTROL ACTION
  
  % this section is used to generate the control action, based on the sensor values

  speed = ones(1,2);      % set both wheel speeds to maximum (1.0) as an initialisation
  
%  if sensor_dist(IR_FRONT) < sensors(IR_FRONT,2)
    % make the maximum speed a fraction of the distance to the nearest
    % obstacle detected by the front IR range finder, relative to the
    % maximum sensor distance for that device
%    speed = speed * sensor_dist(IR_FRONT) / sensors(IR_FRONT,2);
%  end  
  
  % drive down the centre of the hallway
  % . if the sum of the left and right IR range finders is wider than the
  %   typical hallway, it is likely that the vehicle only detects a
  %   single wall
  % . when two walls are seen, drive in the centre
  % . when one wall is seen, drive at a set distance from the wall
  %fprintf('sensor width: %4.1f     hallway width: %4.1f\r', ((sensor_dist(IR_LEFT)+sensors(IR_LEFT,4)) + (sensor_dist(IR_RIGHT)-sensors(IR_LEFT,4))), hallway_width*1.2);
  if ((sensor_dist(IR_LEFT)+sensors(IR_LEFT,4)) + (sensor_dist(IR_RIGHT)-sensors(IR_LEFT,4))) < hallway_width * 1.2  % 120% of the nominal hallway width
    % set opposite-side wheel speeds as a function of the near-side IR
    % sensor distances to the wall
    speed(1) = speed(1) * (sensor_dist(IR_RIGHT) - sensors(IR_RIGHT,6)) / (hallway_width/2 * 1.2);
    speed(2) = speed(2) * (sensor_dist(IR_LEFT) + sensors(IR_LEFT,6)) / (hallway_width/2 * 1.2);
  elseif sensor_dist(IR_LEFT) < sensor_dist(IR_RIGHT)
    % follow the left wall at a nominal distance (hallway_width / 2)
    if sensor_dist(IR_LEFT)+sensors(IR_LEFT,6) < hallway_width/2
      % too close to the left wall - slow down the right-side wheel speed
      speed(2) = speed(2) * (sensor_dist(IR_LEFT) + sensors(IR_LEFT,6)) / (hallway_width/2 * 1.2);
    else
      % too far from the left wall - slow down the left-side wheel speed
      speed(1) = speed(1) * (sensor_dist(IR_LEFT) + sensors(IR_LEFT,6)) / (hallway_width/2 * 1.2);
    end
  else
    % follow the right wall at a nominal distance (hallway_width / 2)
    if sensor_dist(IR_RIGHT)-sensors(IR_RIGHT,6) < hallway_width/2
      % too close to the right wall - slow down the right-side wheel speed
      speed(1) = speed(1) * (sensor_dist(IR_RIGHT) - sensors(IR_RIGHT,6)) / (hallway_width/2 * 1.2);
    else
      % too far from the right wall - slow down the left-side wheel speed
      speed(2) = speed(2) * (sensor_dist(IR_RIGHT) - sensors(IR_RIGHT,6)) / (hallway_width/2 * 1.2);
    end
  end

  %When north wall is reached, turn ccw
  if control_step==1 && sensor_dist(IR_FRONT) < (hallway_width/2) %15
    speed = [-0.4 0.4];
    control_step = 2;
  end
  if control_step==2
    speed = [-0.4  0.4];
  end
  
  %When facing west, go straight
  if control_step==2 && (test_ir_trend(IR_REAR,sensor_hist) + test_ir_trend(IR_RIGHT,sensor_hist)) > 0 && sensor_dist(IR_LEFT) > 36 && sensor_dist(IR_FRONT) > 36
    control_step = 3;
  end
  
  %When left sensor sees floating room wall, advance control step
  if control_step==3 && sensor_dist(IR_LEFT) < 25 && sensor_dist(IR_REAR) > 11
    control_step = 4;
  end
  
  %When doorway is reached, start sweeping turn into floating room
  if control_step==4 && sensor_dist(IR_LEFT) > 20 && sensor_dist(IR_REAR) > 30
    control_step = 5;
  end
  if control_step==5
    speed = [0.2  0.9];
  end
  
  %begin ccw rotation when front sensor sees doorway
  if control_step==5 && sensor_dist(IR_FRONT) < 15
    speed = [-0.4  0.4];
    control_step = 6;
  end
  if control_step==6
    speed = [-0.4  0.4];
  end
  
  %end rotation when facing south
  if control_step==6 && test_ir_trend(IR_REAR,sensor_hist) && sensor_dist(IR_FRONT) > 25
    control_step = 7;
  end
  
  %initiate ccw rotation to locate the candle
  if control_step==7 && sensor_dist(IR_FRONT) < hallway_width/2
    speed = [-0.5  0.5];
    control_step = 8;
  end

  %continue rotation when front sensor sees door
  if control_step==8 && sensor_dist(IR_FRONT) > 30
    speed = [-0.5 0.5];
    control_step = 11;
  end
  if control_step==11
    speed = [-0.5 0.5];
  end

  %when facing north, go straight
  if control_step==11 && (test_ir_trend(IR_RIGHT,sensor_hist)+test_ir_trend(IR_REAR,sensor_hist)) > 0
    control_step = 12;
  end

  %when hallway is reached, begin ccw rotation
  if control_step==12 && sensor_dist(IR_FRONT) < hallway_width/2
    speed = [-0.5 0.5];
    control_step = 13;
  end
  if control_step==13
    speed = [-0.5 0.5];
  end

  %when facing west, go straight
  if control_step==13 && test_ir_trend(IR_RIGHT,sensor_hist) && sensor_dist(IR_REAR) > 20
    control_step = 14;
  end

  %when end of hallway is reahed, begin ccw rotation
  if control_step==14 && sensor_dist(IR_FRONT) < hallway_width/2 && sensor_dist(IR_LEFT) > 36
    speed = [-0.5 0.5];
    control_step = 15;
  end
  if control_step==15
    speed = [-0.5 0.5];
  end

  %when facing south go straight
  if control_step==15 && (test_ir_trend(IR_RIGHT,sensor_hist) + test_ir_trend(IR_REAR,sensor_hist)) > 0 && sensor_dist(IR_LEFT) > 20 && sensor_dist(IR_FRONT) > 20
    control_step = 16;
  end

  %begin large radius turn when t junction is reached
  if control_step==16 && sensor_dist(IR_RIGHT) > 20
    control_step = 17;
  end
  if control_step==17
    speed = [0.9 0.1];
  end

  %begin cw rotation when opposite corner is seen
  if control_step==17 && test_ir_trend(IR_FRONT,sensor_hist)
    speed = [0.5 -0.5];
    control_step = 18;
  end
  if control_step==18
    speed = [0.5 -0.5];
  end

  %when facing west, go straight
  if control_step==18 && test_ir_trend(IR_REAR,sensor_hist) && sensor_dist(IR_FRONT) > 30 % && sensor_dist(IR_LEFT) > 20
    speed = [1 1];
    control_step = 19;
  end

  %when doorway is reached, begin cw rotation
  if control_step==19 && sensor_dist(IR_FRONT) < 10
    speed = [0.5 -0.5];
    control_step = 20;
  end

  %When facing into room, advance control step
  if control_step==20 && sensor_dist(IR_FRONT) > 20
    speed = [0.5 -0.5];
    control_step = 21;
  end
  if control_step==20 || control_step==21
    speed = [0.5 -0.5];
  end

  %When facing north go straight
  if control_step==21 && (test_ir_trend(IR_REAR,sensor_hist) + test_ir_trend(IR_LEFT,sensor_hist)) > 0
    control_step = 22;
  end

  %When fully in room, initiate turn to search for candle
  if control_step==22 && sensor_dist(IR_FRONT) < hallway_width/2
    speed = [0.25 -0.25];
    control_step = 23;
  end

  % continue turning anticlockwise whilst both flame sensors don't see the candle
  if control_step==8 && sensor_dist(FL_LEFT) > sensors(FL_LEFT,2) && sensor_dist(FL_RIGHT) > sensors(FL_RIGHT,2)
    speed = [-0.25  0.25];
  end
  if control_step==23 && sensor_dist(FL_LEFT) > sensors(FL_LEFT,2) && sensor_dist(FL_RIGHT) > sensors(FL_RIGHT,2)
    speed = [0.25  -0.25];
  end
  
  % continue turning anticlockwise whilst one of the flame sensors doesn't see the candle
  if control_step==8 && (sensor_dist(FL_LEFT) < sensors(FL_LEFT,2) || sensor_dist(FL_RIGHT) < sensors(FL_RIGHT,2))
    speed = [-0.25  0.25];
    control_step = 9;
  end
  if control_step==23 && (sensor_dist(FL_LEFT) < sensors(FL_LEFT,2) || sensor_dist(FL_RIGHT) < sensors(FL_RIGHT,2))
    speed = [0.25  -0.25];
    control_step = 9;
  end
  
  % advance the control step if both flame sensors detect the candle
  if sensor_dist(FL_LEFT)<sensors(FL_LEFT,2) && abs(sensor_dist(FL_LEFT)-sensor_dist(FL_RIGHT)) <= 2.0
    control_step = 10;
  end

  % use the flame sensors to steer the robot towards the candle and approach it
  if control_step==9
    speed(1) = (sensors(FL_LEFT,2) - sensor_dist(FL_LEFT)) / sensors(FL_LEFT,2);
    speed(2) = (sensors(FL_RIGHT,2) - sensor_dist(FL_RIGHT)) / sensors(FL_RIGHT,2);
  end
  
  % drive forward towards the candle until the front line detector is trigered
  if control_step==10
    if sensor_dist(LD_FRONT) > 1.0
      speed = [ 0.25  0.25];
    else
      speed = [ 0.00  0.00];
      control_step = 10;
    end
  end
  
  % ADD FURTHER CONTROL ACTION STEPS HERE TO:
  %  -  Exit the floating room
  %  -  Navigate to the northwest room
  %  -  Search for the candle
  %  -
  %  -  ... Search other rooms






  % set the normalised speed values into motor/wheel speeds
  speed = speed * max_speed;
  
  % ensure the motor speed values are between +/-max_speed, with 0.0
  % slightly offset to prevent a 'divide-by-zero' error
  speed(speed<-max_speed) = -max_speed;
  speed(speed==0.0) = 0.01;
  speed(speed>max_speed) = max_speed;
  
% update the odometry values based on the control action wheel speeds
  odometry = odometry + speed;
  
  speed = speed + [rand()-0.5 rand()-0.5]*movement_noise*max_speed;
  
  % print current status of vehicle position, sensor readings, and wheel speeds -- for debugging
%  fprintf('step: %3i   position: (%6.2f,%6.2f)   sensor_l: %6.2f  sensor_r: %6.2f    speed_l: %5.2f  speed_r: %5.2f\n',t/dt,vehicle_pos(1),vehicle_pos(2),sensor_val(1,3),sensor_val(3,3),speed(1),speed(2));
  
  % next, update the vehicle position and orientation, based on the control
  % action
  travel_dist = speed * dt;  % the distance travelled is speed * time
  
  if travel_dist(1) > travel_dist(2)  % vehicle turns to the right
    % arc length 1 = radius 1 * theta
    % arc length 2 = radius 2 * theta
    % radius 1 = radius 2 + wheelbase
    %
    % arc length 1      arc length 2
    % -------------  =  ------------
    % radius 2 + wb       radius 2
    %
    % arc length 1     radius 2 + wb
    % ------------  =  -------------
    % arc length 2       radius 2
    %
    % arc length 1            wb
    % ------------  =  1 + --------
    % arc length 2         radius 2
    %
    %                    ( arc length 1       )
    % radius 2  =  wb  / ( ------------  -  1 )
    %                    ( arc length 2       )
    %
    r2 = wheelbase / ((travel_dist(1)/travel_dist(2)) - 1);  % inner wheel radius
    theta = -travel_dist(2)/r2;                      % angle around a circle that was traversed
    r = -(r2 + wheelbase/2);                         % vehicle centreline radius
  else
    r1 = wheelbase / ((travel_dist(2)/travel_dist(1)) - 1);  % inner wheel radius
    theta = travel_dist(1)/r1;                       % angle around a circle that was traversed
    r = r1 + wheelbase/2;                            % vehicle centreline radius
  end
  
  if speed(1) ~= speed(2)
    crd = 2*r*sin(theta/2);                          % chord length travelled
    heading = vehicle_pos(3) + (90 - (180-(theta*180/pi))/2);  % new vehicle heading (in degrees)
  else
    crd = speed(1);
    heading = vehicle_pos(3);
  end
  
  % ensure that the heading value is between -180 and 180
  if heading < -180; heading = heading + 360; end
  if heading >  180; heading = heading - 360; end
  
  vehicle_pos(1) = vehicle_pos(1) + crd * cos(heading*pi/180);  % new vehicle x-position
  vehicle_pos(2) = vehicle_pos(2) + crd * sin(heading*pi/180);  % new vehicle y-position
  vehicle_pos(3) = heading;                                     % new vehicle heading (in degrees)
  
  % update the vehicle path array
  vehicle_path(computation_cycles+1,:) = vehicle_pos(1:2);
  
  % update the total distance travelled variable
  total_travel_dist = total_travel_dist + abs(crd);
  
  %control_step
  % vehicle_pos  % used to debug updated vehicle position and heading results
  
end  % endre of 't' FOR LOOP

% display final status results to the command window
%fprintf('total distance travelled: %6.2f    odometry: (%6.2f, %6.2f)    final position - x: %5.2f  y: %5.2f  theta: %6.2f\n',total_travel_dist,odometry(1),odometry(2),vehicle_pos(1),vehicle_pos(2),vehicle_pos(3));
%fprintf('computation cycles: %4i    goal reached: %1i     obstacle crashes: %3i    wall crashes: %i\n',computation_cycles,goal_reached,num_crashes,wall_crashes);

end % end of the 'trinity02.m' function



function result = test_ir_trend(ir_label,sensor_hist)

  % this function tests whether the robot has rotated past a position where one of its sensors was
  % perpendicular with an obstacle
  % perpendicularity is determined when the psat three sensor distance values have been decreasing
  % in value whilst the current distance value is hurther than the last distance
  % the funciton will return '1' if this pattern of sensor distance values is met, or '0' otherwise
  result = 0;
  if sensor_hist(1,ir_label) > sensor_hist(2,ir_label) && sensor_hist(2,ir_label) < sensor_hist(3,ir_label) && sensor_hist(3,ir_label) < sensor_hist(4,ir_label)
    result = 1;
  end
  
end













