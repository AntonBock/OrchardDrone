clc; clear all;

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
tree = pcread('orchard.pcd'); %Read point cloud obteined during mapping

%Delete ground points:
X=tree.Location(:,1);
Y=tree.Location(:,2);
Z=tree.Location(:,3);
roi=[min(X) max(X) min(Y) max(Y) 0.1 max(Z)]; %Threshold set to 0.1 m
indices = findPointsInROI(tree,roi);
tree2 = select(tree,indices);
pcshow(tree2);

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Clustering trees. Each tree will be shown with a different colour
X=tree2.Location(:,1);
Y=tree2.Location(:,2);
tree3=pointCloud([X Y zeros(length(X),1)]);
minDistance=0.2; %The points that are within 0.2 meters of each other
                 %are registered within the same cluster
[labels,numClusters] = pcsegdist(tree3,minDistance);
figure
pcshow(tree3.Location,labels);
colormap(hsv(numClusters));
view(2)

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create bounding boxes of the trees
treePoints=tree3.Location;
lab=double(labels);
pointsLab=[lab treePoints];
BBoxes=zeros(numClusters,4);
BBmm=BBoxes;

for i=1:numClusters
    clusIdx=find(pointsLab(:,1)==i);
    cluster=pointsLab(clusIdx,2:4);
    xmin=min(cluster(:,1));
    ymin=min(cluster(:,2));
    w=max(cluster(:,1))-xmin;
    h=max(cluster(:,2))-ymin;
    BBoxes(i,:)=[xmin ymin w h];                                                   
    xmax=xmin+w;
    ymax=ymin+h;
    BBmm(i,:)=[xmin ymin xmax ymax];
    %rectangle('Position', BBoxes(i,:),'EdgeColor','r','LineWidth',2) ;%Show each bounding box
end

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create bounding boxes of the rows
%Merge close bounding boxes of the trees

%%%%Expand BB
th=1.2;
for k = 1 : numClusters
    for j = k : numClusters
         if BBmm(k,1)-BBmm(j,3)< th && BBmm(k,1)-BBmm(j,3)>0 %Horizontal left
            BBmm(k,1)=BBmm(k,1)-th;
        end
        if BBmm(j,1)-BBmm(k,3)< th && BBmm(j,1)-BBmm(k,3)>0 %Horizontal right
            BBmm(j,1)=BBmm(j,1)-th;
        end
        if BBmm(k,2)-BBmm(j,4)< th && BBmm(k,2)-BBmm(j,4)>0 %Vertical top
            BBmm(k,2)=BBmm(k,2)-th;
        end
        if BBmm(j,2)-BBmm(k,4)< th && BBmm(j,2)-BBmm(k,4)>0 %Vertical bottom
            BBmm(j,2)=BBmm(j,2)-th;
        end
    end
end

%%%%Merge overlapping BB

xmin=[];ymin=[];xmax=[];ymax=[];BB=[];
for k = 1 : numClusters
    xmin=[xmin; BBmm(k,1)];
    ymin=[ymin; BBmm(k,2)];
    xmax=[xmax; BBmm(k,3)];
    ymax=[ymax; BBmm(k,4)];
    BB=[BB; BBmm(k,1) BBmm(k,2) BBmm(k,3)-BBmm(k,1) BBmm(k,4)-BBmm(k,2)];
end

% Compute the overlap ratio
overlapRatio = bboxOverlapRatio(BB, BB);

% Set the overlap ratio between a bounding box and itself to zero to
% simplify the graph representation.
n = size(overlapRatio,1); 
overlapRatio(1:n+1:n^2) = 0;

% Create the graph
g = graph(overlapRatio);

% Find the connected text regions within the graph
componentIndices = conncomp(g);

% Merge the boxes based on the minimum and maximum dimensions.
xmin = accumarray(componentIndices', xmin, [], @min);
ymin = accumarray(componentIndices', ymin, [], @min);
xmax = accumarray(componentIndices', xmax, [], @max);
ymax = accumarray(componentIndices', ymax, [], @max);

% Compose the merged bounding boxes using the [x y width height] format.
BBrows = [xmin ymin xmax-xmin ymax-ymin];

%Show rows BBs
for k=1:size(BBrows,1)
    %rectangle('Position', BBrows(k,:),'EdgeColor','y','LineWidth',1) ;
end




%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Generate Waypoints
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Rows waypoints: creates waypoints centered between two rows and placed at
%the beginning and end of each row

rn=size(BBrows,1);%Rows number
off=1.4; %Distance between trees and the drone when there aren't two trees to fly between
WPR=[BBrows(1,1)-off BBrows(1,2)-off; BBrows(k,1)+BBrows(k,3)+off BBrows(1,2)-off]; 

for k=1:rn-1
    if rem(k, 2) == 0 %if the row number is even
        y =(BBrows(k+1,2)+BBrows(k,2)+BBrows(k,4))/2;
        x2=max(BBrows(k,1)+BBrows(k,3),BBrows(k+1,1)+BBrows(k+1,3))+off;
        x1=min(BBrows(k,1),(BBrows(k+1,1)))-off;
        WPR=[WPR; x1 y];
        WPR=[WPR; x2 y];
    end
    if rem(k, 2) == 1 %if the row number is odd
        y =(BBrows(k+1,2)+BBrows(k,2)+BBrows(k,4))/2;
        x1=max(BBrows(k,1)+BBrows(k,3),BBrows(k+1,1)+BBrows(k+1,3))+off;
        x2=min(BBrows(k,1),(BBrows(k+1,1)))-off;
        WPR=[WPR; x1 y];
        WPR=[WPR; x2 y];%Points created following negative direction
    end
end
WPR=[WPR;BBrows(rn,1)+BBrows(rn,3)+off BBrows(rn,2)+BBrows(rn,4)+off; BBrows(rn,1)-off BBrows(rn,2)+BBrows(rn,4)+off];

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Waypoints in front of each tree
[~, s] = sort(BBoxes(:, 1));
TBB2=BBoxes(s, :);
[~, s] = sort(TBB2(:, 2));
TBB=TBB2(s, :);
WPT=[(TBB(:,1)+TBB(:,1)+TBB(:,3))./2 TBB(:,2)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Generate path the drone should follow
tpr=length(TBB)/rn;%number of trees per row
WP=[WPR(1,:)];%Initial waypoint
for k=1:rn
    if rem(k, 2) == 0%if the row number is even
       TBB2=flipud(WPT(1+(k-1)*tpr:tpr*k,:));
       WP=[WP;  TBB2(:,1) TBB2(:,2)-off];
       %Points created following negative direction
    end
    if rem(k, 2) == 1%if the row number is odd
       WP=[WP; WPT(1+(k-1)*tpr:tpr*k,1) WPT(1+(k-1)*tpr:tpr*k,2)-off];
       %Points created following positive direction
    end
    if k <= rn
       WP=[WP;WPR(k*2,:);WPR(k*2+1,:)];
    end
end
TBB2=flipud(WPT(1+(rn-1)*tpr:tpr*rn,:));
WP=[WP;  TBB2(:,1) TBB2(:,2)+BBrows(rn,4)+off];
WP=[WP;WPR(end,:)];%Final waypoint

%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Show waypoints and path
hold on
plot3(WP(:,1), WP(:,2),zeros(length(WP),1),'-','Color',[0,0.5,1],'Linewidth',2.5) %Show path
pcshow([WP(:,1) WP(:,2) zeros(length(WP),1)], ".r", "MarkerSize", 200) %Show waypoints
view(2)
%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Determine tree center position
 treePos=[];
 for i=1:size(BBoxes,1)
     treePos=[treePos; (BBoxes(i,1)*2+BBoxes(i,3))/2 (BBoxes(i,2)*2+BBoxes(i,4))/2];
 end


%% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Create excel file of the waypoints
filename = 'row_waypoints.xlsx';
variables=["X" "Y"];
writematrix(variables,filename,'Sheet',1,'Range','A1')
writematrix(WP,filename,'Sheet',1,'Range','A2')