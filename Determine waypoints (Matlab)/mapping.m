clc; clear all;
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%This code finds the waypoints for mapping and create the path to cover the
%entire orchard

%INPUTS needed:
fieldSize=[24 11.5]; %Field size X by Y [m]
gimbal_angle=90; %Angle of the camera gimbal respect horizontal axis [deg]
FOV_h=70; %Horizontal field of view of the camera [deg]
h_tree_max=2;%Height of the tallest tree [m]

%------%
h_drone=h_tree_max+0.5;%Height of the drone [m]. Set to fly 0.5 m above the tallest tree
step=2*tan(deg2rad(FOV_h))*(h_drone-h_tree_max)/(sin(deg2rad(gimbal_angle)));%Distance between points
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Adapt the size of the field to the step size
fieldSize2=sort(fieldSize,'descend');
fsX=ceil((fieldSize2(1)-step/2)/step)*step; %Rounding X size of the field
fsY=ceil((fieldSize2(2)-step/2)/step)*step; %Rounding Y size of the field
rows=fsX/step+1;
columns=fsY/step+1;

%Inicialize variables
WPx=step/2;
WPy=0;
waypoints=zeros(rows*columns,2);


%Determining the points and creating the path:

for i=1:columns %Number of columns of the point grid
    if rem(i,2)==1 %if the number of the column is odd
        for j=1:rows
            waypoints((i-1)*rows+j,:)=[WPx WPy]; %Row created following positive direction
            if WPy+step>fieldSize(1)
                WPy=fieldSize(1);
            else
                WPy=WPy+step; %next point
            end
        end
    elseif rem(i,2)==0 %if the number of the column is even
        for j=rows:-1:1
            waypoints((i-1)*rows+j,:)=[WPx WPy]; %Row created following negative direction
            if WPy+step>fieldSize(1)
                WPy=fieldSize(1);
            else
                WPy=WPy+step; %next point
            end
        end
    end
    WPy=0; %Reinicialize Y position
    if WPx+step>fieldSize(2)
        WPx=fieldSize(2);
    else
        WPx=WPx+step; %next row
    end
end

 if fieldSize(1)>fieldSize(2)
     waypoints=fliplr(waypoints);
 end
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Show result

figure
xlabel('X [m]');
ylabel('Y [m]');
hold on;
axis([-step fsX+step -step fsY+step]);
grid on;


waypoints=[waypoints ones(length(waypoints),1)*h_drone];
rectangle('Position',[0 0 fieldSize]);
plot(waypoints(:,1), waypoints(:,2), '-','Color',[0,0.5,1],'Linewidth',2.5)
plot(waypoints(:,1), waypoints(:,2), ".r", "MarkerSize", 20)
xlim=[-1 25];
ylim=[-1 13];
axis equal
view(2)



%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create excel file of the waypoints
filename = 'mapping_waypoints.xlsx';
variables=["X" "Y" "Z"];
writematrix(variables,filename,'Sheet',1,'Range','A1')
writematrix(waypoints,filename,'Sheet',1,'Range','A2')

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Plot our simulation environment trees. This step is not necessary, it can be removed


radious=1;
treeNum=15;
center=[2 5];
for i=1:treeNum
    viscircles(center, radious,'Color','b');
    if rem(i,5)==0
        center(1)=2;
        center(2)=center(2)+5;
    else
        center(1)=center(1)+4;
    end
end




%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation of how the drone will follow the path. Step not necessary,
% visualizing purpose.

toa=0:length(waypoints)-1;
trajectory = waypointTrajectory(waypoints,toa);

orientationLog = zeros(toa(end)*trajectory.SampleRate,1,'quaternion');
count = 1;
while ~isDone(trajectory)
   [currentPosition,orientationLog(count)] = trajectory();

   plot3(currentPosition(1),currentPosition(2),currentPosition(3),'.g')

   pause(trajectory.SamplesPerFrame/trajectory.SampleRate)
   count = count + 1;
end

hold off