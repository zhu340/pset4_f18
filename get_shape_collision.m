function collision = get_shape_collision(state,R,obs_list)
%state ; [x,y,theta] state of robot
%R:radius of robot,or 115mm
collision = 0;

for row = 1:size(obs_list,1)
    obs = obs_list(row,:);
    w_2 = obs(3)/2;
    h_2 = obs(4)/2;
    
    distx = abs(state(1) - (obs(1)+w_2));
    disty = abs(state(2) - (obs(2)+h_2));
    
    if distx > w_2 + R || disty > h_2 + R
        collision = 0;
    elseif distx <= w_2 || disty <= h_2
        collision = 1;
    else
        dist = (distx - w_2)^2 + (disty - h_2)^2;
        
        if dist <= R^2
            collision = 1;
        else
            collision = 0;
        end
    end
    
    if collision == 1
        break;
    end
    
    
end
end