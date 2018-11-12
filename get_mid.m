function [mid_node,idx] = get_mid(path,mid_pt)
%path:Nx3 matrix with each row represents a node on the path

path1 = path(2:end,:);
path2 = path(1:end-1,:);

x1 = path1(:,1);
x2  = path2(:,1);

y1 = path1(:,2);
y2 = path2(:,2);


dists = sqrt((x1-x2).^2 + (y1-y2).^2);


for i = 1:size(dists,1)
    dist = sum(dists(1:i));
    if dist >= mid_pt
        if i == 1
            idx =1;
        else
        idx = i-1;
        end
        lastLen = dists(idx);
        break
    end
end

if idx == 0 || idx > length(dists)
    mid_node = [0,0,0];
else
    ratio = (dist - mid_pt)/lastLen;
    
    x = path(idx,1) + (path(idx+1,1) - path(idx,1))*ratio;
    y = path(idx,2) + (path(idx+1,2) - path(idx,2))*ratio;
    theta = path(idx,3);
    
    mid_node = [x,y,theta];
end
end