function len = get_path_length(path)
%path:Nx3 matrix with each row represents a node on the path

path1 = path(2:end,:);
path2 = path(1:end-1,:);

x1 = path1(:,1);
x2  = path2(:,1);

y1 = path1(:,2);
y2 = path2(:,2);


len = sum(sqrt((x1-x2).^2 + (y1-y2).^2));
end

