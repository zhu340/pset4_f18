% function [U,Nodes] = get_traj(init,target,r,d,dt,xMax,yMax)
%init:Initial state
%traget:target state
%r: wheel radius or 20
%d: robot width or 85
%dt: sampling time
%xMax,yMax: map boundries
%U: Nx2 matrix representing set of control inputs with each row being one u
%traj: set of trajectory with each row being one x
clc
clear all

init = [2000,1420,-pi/2];
target = [3000,280,pi/2];
r = 20;
d = 85;
xMax = 5580;
yMax = 1500;


sample_rate = 5;
expand_dis = 1;

Nodes(1).value = init;
Nodes(1).parent = 0;

t = 2;

%% Find the path

while 1
    if randi([0,100]) > sample_rate
        random_node = [randi([0,xMax]),randi([0,yMax]),randi([-180,180])/pi];
    else
        random_node = target;
    end
    
    %expand tree
    [idx,closest] = get_closest(vertcat(Nodes.value),random_node);
    
    theta = atan2(random_node(2) - closest(2), random_node(1) - closest(1));
    
    value = [closest(1) + expand_dis*cos(theta), closest(2) + expand_dis *sin(theta) theta];%!!!!!!!!!!!!!
    
    parent = closest;
    
    Nodes(t).value = value;
    Nodes(t).parent = idx;
    t = t+1;
    
    if sqrt((value(1) - target(1))^2 + (value(2) - target(2))^2) < 2 && abs(value(3)-target(3)) < (1/3 * pi)
        fprintf('reached.\n')
        break
    end
    
end

%% Inverse the tree search

path = Nodes(end).value;
idx =  Nodes(end).parent;

for i = 1:length(Nodes)
    path = [path; Nodes(idx).value];
    idx = Nodes(idx).parent;
    
    if idx == 0 
        break;
    end
end

path = flipud(path);

%% Smooth path
newpath = path;
path_len = get_path_length(newpath);
NIter = 1500;

for i = 1:NIter
    pts = [randi([1,ceil(path_len)-1]),randi([1,ceil(path_len)-1])];
    pt1 = min(pts);
    pt2 = max(pts);
    
    [pt1,idx1] = get_mid(newpath,pt1);
    [pt2,idx2] = get_mid(newpath,pt2);
    
    
    if idx1 <= 1 || idx2 <= 1 || idx2>size(path,1) || idx2 == idx1
        continue
    end
    
    newpath_temp = [newpath(1:idx1,:);pt1;pt2;newpath(idx2:end,:)];
    len_temp  = get_path_length(newpath_temp);
    if len_temp <= path_len
        path_len = len_temp;
        newpath = newpath_temp;
    end
end


%% Get input u
dt = 1/size(newpath,1);
U = [];
for i = 1:size(newpath,1)-1
    x = newpath(i,:);
    x_nxt = newpath(i+1,:);
    u = inv_move(x,x_nxt,r,d,dt);
    if i == 1
        U = [U;[u,dt]];
    elseif (u(1) - U(end,1))>0.01 || (u(2) - U(end,2))>0.01
        U = [U;[u,dt]];
    else 
        U(end,3) = U(end,3) + dt;
    end
end


% end


%[~,traj] = get_traj([2000,1420,-pi/2],[3000,280,pi/2],0,0,0,5580,1500);