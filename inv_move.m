function u = inv_move(x,x_nxt,r,d,dt)
%xt
x0 = x(1);
y0 = x(2);
theta0 = x(3);

x1 = x_nxt(1);
y1 = x_nxt(2);
theta1 = x_nxt(3);

delta_theta = theta1-theta0;
delta_s = sqrt((x1-x0)^2 + (y1-y0)^2);

%xt+1
temp1 = 2*delta_s/r/dt;
temp2 = d*delta_theta/r/dt;

omega_r = (temp1+temp2)/2;
omega_l = (temp1-temp2)/2;
u = [omega_r,omega_l];

end
