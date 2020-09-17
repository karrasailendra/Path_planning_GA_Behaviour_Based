% Author Sailendra Karra, Yashwanth Surapaneni, Anusha Kilari
% This code is for complex systems final project
% This code is for running swarm of 20 robots in 50*50 area
% Obstacles are predefined, No input from user is required


clc 
clear all

num = 20; % number of robots
Initial_Positions = 50*rand(num,2); % Random generation of robots in entire plot area

% If robot spawned in obstacle area
for k=1:num
if Initial_Positions(k,1)>25 && Initial_Positions(k,1)<30 && Initial_Positions(k,2)>20 && Initial_Positions(k,2)<35
    Initial_Positions(k,:)=rand(1,2);
end
end

% Obstacle reagion
Obstacle_1 = [25 30 30 25 25; 20 20 35 35 20];
Obstacles = [15 15;35 40];

V = 0.01;
dt = 1;
% parameters for weightage
am=1.1;
bm=4;
ar=2;
clearance=1;
timesteps = 850;

waypoints = [5 3;  40 45]; % Initial and final goal positions

% Flocking towards centre of mass
for i = 1:1:2
    X_goal = waypoints(i,1);
    Y_goal = waypoints(i,2);
for j = 1:1:timesteps  
    Updated_Coordinates = [];
    if j == 1
        for z = 1:1:num
             X_curr = Initial_Positions(z,1);
             Y_curr = Initial_Positions(z,2);
             Update = goal_seeking(X_goal,Y_goal,X_curr,Y_curr,dt,am);
             Updated_Coordinates = [Updated_Coordinates;Update];
        end
    else
    for l = 1:1:num
%         if l>1
            near_bot=[];
            Update=Initial_Positions(l,:);
            mindist=[];
            nearest_robot_form=inf;
            for p=1:num
                 distance1=nearest_robot(Update,Initial_Positions(p,:));
                 heading_diff= (Initial_Positions(l,3)- Initial_Positions(p,3));
                     if distance1<nearest_robot_form
                         nearest_bot=Initial_Positions(p,:);
                         nearest_robot_form=distance1;
                     end
                 if distance1<clearance && distance1~=0 
                     mindist=[distance1 mindist];
                     if min(mindist)== distance1
                     near_bot=Initial_Positions(p,:);
                     end
                 end
            end            
             goal_distance=nearest_robot(waypoints(i,:),Initial_Positions(l,:));
             
             % parameters for goal seeking behavior 
             if goal_distance<clearance
                 f1=am*(goal_distance)/bm;
             else
                 f1=am;
             end
             if ~isempty(near_bot)
                 nearbot_distance=nearest_robot(near_bot(1,:),Initial_Positions(l,:));
                 f4=ar*nearbot_distance;
             else
                 f4=0;
             end
             
        if ~isempty(near_bot)  % If no robot is with in safe distance
             X_curr = Initial_Positions(l,1);
             Y_curr = Initial_Positions(l,2);
             heading1=Initial_Positions(l,3);
             heading2=near_bot(1,3);
             goal_distance=nearest_robot(waypoints(i,:),Initial_Positions(l,:));
             if i == 1
             formation_direction = pi/4;
             else
                 formation_direction = atan2((waypoints(i-1,2) - waypoints(i,2)),(waypoints(i-1,1) - waypoints(i,1)));
             end

             Update = [-X_curr -Y_curr 0]+goal_seeking(X_goal,Y_goal,X_curr,Y_curr,dt,f1)+obs_avoid(X_goal,Y_goal,X_curr,Y_curr,dt)+collision_avoid(near_bot(1,1),near_bot(1,2),X_curr,Y_curr,dt,f4)+formation_maintain(nearest_bot(1,1),nearest_bot(1,2),X_curr,Y_curr,dt,heading1,heading2,clearance,formation_direction);
             Updated_Coordinates = [Updated_Coordinates;Update];
        else
             X_curr = Initial_Positions(l,1);
             Y_curr = Initial_Positions(l,2);

             Update = obs_avoid(X_goal,Y_goal,X_curr,Y_curr,dt)+goal_seeking(X_goal,Y_goal,X_curr,Y_curr,dt,f1);

             Updated_Coordinates = [Updated_Coordinates;Update];
        end

    end
        

    end

    Initial_Positions = Updated_Coordinates;

    figure(1)
    grid on
    plot(Initial_Positions(:,1),Initial_Positions(:,2),'.','MarkerSize',15)
    hold on
    plot(Obstacles(:,1) ,Obstacles(:,2),'.','MarkerSize',80);
    hold on
    plot(waypoints(:,1),waypoints(:,2),'*','MarkerSize',6);
    hold on
    plot(Obstacle_1(1,:) ,Obstacle_1(2,:),'k-');
    hold off
    xlim([0 50])
    ylim([0 50])
end
    
end
  
%% function for behavioral goal seeking 
function Update = goal_seeking(X_goal,Y_goal,X_curr,Y_curr,dt,f1)

heading = atan2((Y_goal - Y_curr),(X_goal - X_curr));
 V_mag_sqr = (Y_goal-Y_curr)^2 + (X_goal-X_curr)^2; 
 V = f1*[0.1/sqrt(V_mag_sqr)]*abs([(X_goal-X_curr) Y_goal-Y_curr]);
X_update =  X_curr + V(1)*cos(heading)*dt;
Y_update =  Y_curr + V(2)*sin(heading)*dt;
Update = [X_update Y_update heading];


end

%% function for behavioral collision_avoidance 
function Update = collision_avoid(X_goal,Y_goal,X_curr,Y_curr,dt,f4)

heading = atan2((Y_goal - Y_curr),(X_goal - X_curr));
 V_mag_sqr = (Y_goal-Y_curr)^2 + (X_goal-X_curr)^2; 
 V = f4*[0.1/sqrt(V_mag_sqr)]*abs([(X_goal-X_curr) Y_goal-Y_curr]);
X_update = X_curr + V(1)*cos(heading+2*pi/2)*dt;
Y_update = Y_curr + V(2)*sin(heading+2*pi/2)*dt;
Update = [X_update Y_update heading];
end

%% function for behavioral obstacle_avoidance 
function Update = obs_avoid(X_goal,Y_goal,X_curr,Y_curr,dt)
obstacle1=[35 40];
danger1=nearest_robot(obstacle1,[X_curr Y_curr]);
obstacle2=[15 15];
danger2=nearest_robot(obstacle2,[X_curr Y_curr]);
if X_curr>23 && X_curr<32 && Y_curr>18 && Y_curr<37
    k=3*(X_curr)-(Y_curr)-55;
    f2=1;
    x_diff1=X_curr-25;
    x_diff2=X_curr-30;
    y_diff2=Y_curr-20-7.5;
    y_diff1=Y_curr-35;
    y_diff3=Y_curr-20;
    fac=3;
        fac_x=0;
        fac_y=0;
    theta=0;
    heading = atan2((Y_goal - Y_curr),(X_goal - X_curr));
    if x_diff1<0
        if heading > 0
        fac_x=-1;
        fac_y=1;
        else
        fac_x=-1;
        fac_y=-1;
        end
    end
        if x_diff2>0
        if heading > 0
        fac_x=1;
        fac_y=1;
        else
        fac_x=1;
        fac_y=-1;
        end
        end
        if  y_diff3 <0 && x_diff2<0 
        if heading > 0
        fac_x=1;
        fac_y=-1;
        else
        fac_x=-1;
        fac_y=-1;
        end
        end
        if x_diff1>0 && y_diff1 >0 && x_diff2<0
        if heading < -pi/2   
        fac_x=-1;
        fac_y=1;
        else
        fac_x=1;
        fac_y=1;
        end
        end
V_mag_sqr = (Y_goal-Y_curr)^2 + (X_goal-X_curr)^2;
V = f2*fac*[0.1/sqrt(V_mag_sqr)]*abs([(X_goal-X_curr) Y_goal-Y_curr]);
if heading < 0.01 && heading > -0.01
X_update =  fac_x*0.1;
Y_update =  2;
else
X_update =  fac_x*0.1;
Y_update =  fac_y*0.1;
end
elseif  danger1 < 3.5
        f2=2;
    obstacle=obstacle1;
    heading = atan2((Y_goal - Y_curr),(X_goal - X_curr));
 V_mag_sqr = (Y_goal-Y_curr)^2 + (X_goal-X_curr)^2; 
 V = f2*[0.1/sqrt(V_mag_sqr)]*abs([(X_goal-X_curr) Y_goal-Y_curr]);
X_update =  V(1)*cos(heading+pi/2)*dt;
Y_update =   V(2)*sin(heading+pi/2)*dt;
elseif  danger2 < 3.5
           f2=2;
    obstacle=obstacle2;
    heading = atan2((Y_goal - Y_curr),(X_goal - X_curr));
    heading1 = atan2((obstacle(2) - Y_curr),(obstacle(1) - X_curr));
 V_mag_sqr = (Y_goal-Y_curr)^2 + (X_goal-X_curr)^2; 
 V = f2*[0.1/sqrt(V_mag_sqr)]*abs([(X_goal-X_curr) Y_goal-Y_curr]);
 if heading>heading1 
X_update =  V(1)*cos(heading+pi/2)*dt;
Y_update =   V(2)*sin(heading+pi/2)*dt;
 elseif heading < -3*pi/4
     X_update =  V(1)*cos(heading+pi/2)*dt;
Y_update =   V(2)*sin(heading+pi/2)*dt;
 else
      X_update =  V(1)*cos(heading-pi/2)*dt;
Y_update =   V(2)*sin(heading-pi/2)*dt;
 end
else
    X_update = 0;
    Y_update = 0;
    heading = 0;
end

Update = [X_update Y_update heading];
end
%% function for formation
function Update = formation_maintain(X_goal,Y_goal,X_curr,Y_curr,dt,heading1,heading2,clearance,formation_direction)
 ak=0.5;

 V_mag_sqr = (Y_goal-Y_curr)^2 + (X_goal-X_curr)^2; 
 X_des= Y_goal+(V_mag_sqr*cos(heading2-heading1+formation_direction));
 Y_des= Y_goal+(V_mag_sqr*sin(heading2-heading1+formation_direction));
  if nearest_robot([X_des Y_des],[X_curr Y_curr])< 3 % clearance
      f5=ak*nearest_robot([X_des Y_des],[X_curr Y_curr]);
  else
      f5=0;
  end
 V_mag_sqr = (Y_des-Y_curr)^2 + (X_des-X_curr)^2; 
 V = f5*[0.1/sqrt(V_mag_sqr)]*abs([(X_goal-X_curr) Y_goal-Y_curr]);
 heading = atan2((Y_des - Y_curr),(X_des - X_curr));
X_update = + V(1)*cos(heading)*dt;
Y_update = - V(2)*sin(heading)*dt;
Update = [X_update Y_update heading];


end

%% function for distance between 2 points
    function distance1 = nearest_robot(rob1,rob2)

       distance1=sqrt((rob1(1)-rob2(1))^2+(rob1(2)-rob2(2))^2);
       

end
        
