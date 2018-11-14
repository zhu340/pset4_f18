function [idx,min_pt,min_dist] = get_closest(V,target)
%V is a Nx3 with each row [x,y,theta] represents the state of the point
%target is 1x2 with [x,y] reprsents the location
%Return both the minmum distance and the point

dists = zeros(size(V,1),1);

tx = target(1);
ty = target(2);

for r = 1:size(V,1)
    dists(r) = (V(r,1)-tx)^2 + (V(r,2)-ty)^2;
end


[min_sqr, idx] = min(dists);

min_dist = sqrt(min_sqr);
min_pt = V(idx,:);

end