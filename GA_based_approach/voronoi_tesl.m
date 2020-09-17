% This function is used to find network graph by using Voronoi tessellation
% Inputs: Obstacles, Wall are given to the function
% Outputs: distances, V_new, Ind_mins,Ind_ming are the final distances
% matrix and final coordinates in network graph which will be used in
% Dijkstra's algorithm.

function [distances,V_new,Ind_mins,Ind_ming]=voronoi_tesl(Obstacles,Wall)

% Obtaining size of the cell
[row col] = size(Obstacles);

%% Identifying the limits of obstacles in X and Y directions
X_limit = []; Y_limit = [];
for i = 1:1:row
    x_coordinates = Obstacles{i}(:,1);
    x_coordinates = unique(x_coordinates,'sorted');
    x_coordinates = x_coordinates';
    X_limit = [X_limit;x_coordinates];
    y_coordinates = Obstacles{i}(:,2);
    y_coordinates = unique(y_coordinates,'sorted');
    y_coordinates = y_coordinates';
    Y_limit = [Y_limit;y_coordinates];
end
%% Combining all Obstacles and walls to generate voronoi
X = [];
for i = 1:1:row
    X = [X;Obstacles{i}];
end

for i = 1:1:4
    X = [X;Wall{i}];
end

%% Obtaining the vertices of edges of voronoi 
[vx,vy] = voronoi(X(:,1),X(:,2));
[V,C] = voronoin(X);
V(1,:) = [];
[row_v col_v] = size(V);
%% Collecting Vertices which are outside of obstacles
V_new = [];
for i =1:1:row_v
    count = 0;
    for j=1:1:row
        if V(i,1)>=X_limit(j,1) && V(i,1)<=X_limit(j,2) && V(i,2)>=Y_limit(j,1) && V(i,2)<=Y_limit(j,2)
            count = count+1;
        end
    end
    if count==0
        V_coordinates = [V(i,1) V(i,2)];
        V_new = [V_new;V_coordinates];
    end
end

% plot(V_new(:,1),V_new(:,2),'.r');
% 
node_array=V_new';

%% Finding nodes(Structure) i)nodes_coordinate ii)node_children that are connected
xy_node=[];
empty_nodes.Cordinate=[];
empty_nodes.nodes_connected=[];
empty_nodes.nodes_parent=[];
empty_nodes.nodes_children=[];
empty_nodes.distances=[];

% Big Nodes Matrix
nodes_co=repmat(empty_nodes,length(node_array),1);
for j=1:length(node_array(1,:))
    
    nodes_co(j).Cordinate = node_array(:,j);
    nodes_co(j).nodes_parent = [nodes_co(j).nodes_parent, j];
    for i=1:length(vx(1,:))
        x1_node = vx(1,i);
        y1_node = vy(1,i);
        xy1_node = [x1_node;y1_node];
        
        x2_node = vx(2,i);
        y2_node = vy(2,i);
        xy2_node = [x2_node;y2_node];
        
        xy_node = [x1_node,x2_node;y1_node,y2_node];
        if ~isequal(xy1_node,xy2_node)
            if(any(all(round(xy_node,5)==round(node_array(:,j),5))))
                connected_nodes = xy_node*~(all((round(xy_node,5)==round(node_array(:,j),5))))';
                nodes_co(j).nodes_connected = [nodes_co(j).nodes_connected, connected_nodes];
                [Id_r, Id_c] = find(all(round(connected_nodes,5)==round(node_array,5)));
                if(~isempty(Id_c))
                  nodes_co(j).nodes_children = [nodes_co(j).nodes_children, Id_c(1)];
                end
            else
                continue;
            end
        end
        
    end
end
%% Finding the distances of each edge of the voronoi tessellation

for i=1:length(nodes_co)
    for j=1:length(nodes_co(i).nodes_children)
        node_index=nodes_co(i).nodes_children(j);
        connected_node=nodes_co(node_index).Cordinate;
        distances_children=sqrt(((nodes_co(i).Cordinate(1)-connected_node(1))^2)+((nodes_co(i).Cordinate(2)-connected_node(2))^2));
        nodes_co(i).distances=[nodes_co(i).distances, distances_children];
    end
end
 
%% creating a matrix of distances
 for i=1:length(nodes_co)
     count=1;
     for j=1:length(nodes_co)
         child_nodes=sort( nodes_co(i).nodes_children);
         if(~isequal(i,j))
             
             if((j==child_nodes(count)))
               distances(i,j)=nodes_co(i).distances(count);
               count=count+1;
               if(count>length(nodes_co(i).nodes_children))
                   count=1;
               end
             else
               distances(i,j)=inf;%   ------------------------------------------------  
             end
         else
             distances(i,j)=0;
         end
     end
 end
 %% Adding the source and destination points to the complete network graph
 source=[5 3];
 goal=[40 45];
 mins=10^6;
 for i=1:length(V_new)
     dista=sqrt((source(1)-V_new(i,1))^2+(source(2)-V_new(i,2))^2);
     if(dista<mins)
         mins=dista;
         Ind_mins=i;
     end
 end
  ming=10^6;
  for i=1:length(V_new)
     dista=sqrt((goal(1)-V_new(i,1))^2+(goal(2)-V_new(i,2))^2);
     if(dista<ming)
         ming=dista;
         Ind_ming=i;
     end
  end

 
% keyboard
%% Identifying if an edge intersects the obstacle and collecting all edges that do not intersect with obstacles
% [r c] = size(vx);
% vx_new = [];
% vy_new = [];
%  for i = 1:1:c % To collect each vertex
%      count = 0;
%   for j = 1:1:row % To check in each obstacle
%       if   vx(1,i)>=X_limit(j,1) && vx(1,i)<=X_limit(j,2)&& vy(1,i)>=Y_limit(j,1) && vy(1,i)<=Y_limit(j,2)...
%          ||vx(2,i)>=X_limit(j,1) && vx(2,i)<=X_limit(j,2)&& vy(2,i)>=Y_limit(j,1) && vy(2,i)<=Y_limit(j,2)...
%          ||vx(1,i)<0||vy(1,i)<0||vx(2,i)<0||vy(2,i)<0||vx(1,i)>50||vy(1,i)>50||vx(2,i)>50||vy(2,i)>50
%           count = count+1;
%       end
%   end
%   if count==0
%       vx_coordinates = [vx(1,i);vx(2,i)];
%       vx_new = [vx_new vx_coordinates];
%       vy_coordinates = [vy(1,i);vy(2,i)];
%       vy_new = [vy_new vy_coordinates];
%   end
%  end
%    [row_vnew col_vnew] = size(V_new);                                      
% %% Plotting Voronoi and it's vertices
% figure(1)
% voronoi(X(:,1),X(:,2))
% xlim([0 50])
% ylim([0 50])
% figure(2)
% plot(vx,vy,'r')
% xlim([0 50])
% ylim([0 50])
% figure(3)
% plot(vx_new,vy_new,'r')
% xlim([0 50])
% ylim([0 50])
% hold on
% labels = cellstr( num2str([1:row_vnew]') );
% scatter(V_new(:,1),V_new(:,2))
% text(V_new(:,1),V_new(:,2),labels,'VerticalAlignment','bottom',...
%                              'HorizontalAlignment','right');
end

