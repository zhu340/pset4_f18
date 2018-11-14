function collision = get_traj_collision(pt1,pt2,obs_list,R)
% state ; [x,y,theta] state of robot
% R:radius of robot,or 115mm


collision = 0;

for row = 1:size(obs_list,1)
    obs = obs_list(row,:);
    x = obs(1);y = obs(2);w = obs(3)+ R;h=obs(4)+R;
    
    if x >= R
        x = x - R; w = w+ R;
    end
    if y >= R
        y = y - R; h = h+ R;
    end
    
    left = get_line_collision(pt1,pt2,[x,y],[x,y+h]);
    right = get_line_collision(pt1,pt2,[x+w,y],[x+w,y+h]);
    up = get_line_collision(pt1,pt2,[x,y+h],[x+w,y+h]);
    down = get_line_collision(pt1,pt2,[x,y],[x+w,y]);
    
    collision = int8(left == 1 || right == 1 || up == 1 || down == 1);
    if collision == 1
        break;
    end
    
end
end



function collision = get_line_collision(s11,s12,s21,s22)
x1 = s11(1);
x2 = s12(1);
y1 = s11(2);
y2 = s12(2);

x3 = s21(1);
x4 = s22(1);
y3 = s21(2);
y4 = s22(2);

uA = ((x4-x3)*(y1-y3) - (y4-y3)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));
uB = ((x2-x1)*(y1-y3) - (y2-y1)*(x1-x3)) / ((y4-y3)*(x2-x1) - (x4-x3)*(y2-y1));

if uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1
    collision = 1;
else
    collision = 0;
end
end