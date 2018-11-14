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
close all

init = [2100,1400,-pi/2];
target = [3100,320,pi/2];
r = 20;
d = 115;
R = 115/2;
xMax = 5580;
yMax = 1500;


sample_rate = 30;
expand_dis = 25;

Nodes(1).value = init;
Nodes(1).parent = 0;

t = 2;


%% Get obstacles
obs_list = zeros(17,4);
obs_list(1,:)  = [0,0,2180,80];
obs_list(2,:)  = [2480,0,500,80];
obs_list(3,:)  = [3500,0,2080,80];
obs_list(4,:)  = [0,0,80,460];
obs_list(5,:)  = [0,660,80,840];
obs_list(6,:)  = [0,1420,1980,80];
obs_list(7,:)  = [280,180,1700,280];
obs_list(8,:)  = [280,660,1700,180];
obs_list(9,:)  = [280,1040,1700,180];
obs_list(10,:) = [2180,660,1700,180];
obs_list(11,:) = [2180,1040,1700,180];
obs_list(12,:) = [2180,1420,3400,80];
obs_list(13,:) = [2180,280,500,180];
obs_list(14,:) = [2680,280,280,100];
obs_list(15,:) = [3200,280,200,100];
obs_list(16,:) = [5490,0,90,840];
obs_list(17,:) = [5490,960,90,540];
run('get_map.m');
hold on;

%% Find the path
while 1
    if randi([0,100]) > sample_rate
        random_node = [randi([0,xMax]),randi([0,yMax]),randi([-180,180])/pi];
    else
        random_node = target;
    end
    
    if get_shape_collision(random_node,R,obs_list)
        continue
    end
    
    
    %expand tree
    [idx,closest,min_dist] = get_closest(vertcat(Nodes.value),random_node);
    
    if min_dist <= expand_dis
        value = random_node;
    else
        theta = atan2(random_node(2) - closest(2), random_node(1) - closest(1));
        
        value = [closest(1) + expand_dis*cos(theta), closest(2) + expand_dis *sin(theta) theta];%!!!!!!!!!!!!!
    end
    
    parent = idx;
    
    if get_shape_collision(value,R,obs_list)
        continue
    end
    
    Nodes(t).value = value;
    Nodes(t).parent = parent;
    t = t+1;
    
    %% Plot evolution of tree
    plot([value(1),Nodes(parent).value(1)],[value(2),Nodes(parent).value(2)],'b-');
    pause(0.001)
    
    %% Check goal
    if sqrt((value(1) - target(1))^2 + (value(2) - target(2))^2) < 2 %%&& abs(value(3)-target(3)) < (1/3 * pi)
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
    
    
    if get_traj_collision(pt1,pt2,obs_list,R) || get_traj_collision(pt1,newpath(idx1,:),obs_list,R) || get_traj_collision(pt2,newpath(idx2,:),obs_list,R)
        continue
    end
    
    newpath_temp = [newpath(1:idx1,:);pt1;pt2;newpath(idx2+1:end,:)];
    len_temp  = get_path_length(newpath_temp);
    if len_temp <= path_len
        path_len = len_temp;
        newpath = newpath_temp;
    end
end

newpath(1,3) = init(3);
newpath(end,3) = target(3);

for i = 2:size(newpath,1)-1
    newpath(i,3) = atan2(newpath(i,2) - newpath(i-1,2), newpath(i,1) - newpath(i-1,1));
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
        %     elseif abs(u(1)/u(2) - U(end,1)/U(end,2))>0.001
    elseif abs(u(1) - U(end,1))>0.001 || abs(u(2) - U(end,2))>0.001
        U = [U;[u,dt]];
    else
        U(end,3) = U(end,3) + dt;
    end
end


% end

% run('test.m');
% hold on;
plot(path(:,1),path(:,2),'c')
plot(newpath(:,1),newpath(:,2),'r')

%[~,traj] = get_traj([2000,1420,-pi/2],[3000,280,pi/2],0,0,0,5580,1500);