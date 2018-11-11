function [min_dist,min_pt] = get_closest(V,target)
%V is a Nx2 with each row [x,y] represents the location of the point
%target is 1x2 with [x,y] reprsents the location
%Return both the minmum distance and the point

dists = zeros(length(V),1);

tx = target(1);
ty = target(2);

for r = 1:length(V)
    dists(r) = (V(r,1)-tx)^2 + (V(r,2)-ty)^2;
end


[min_sqr, idx] = min(dists);

min_dist = sqrt(min_sqr);
min_pt = V(idx,:);

end