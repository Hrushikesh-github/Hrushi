function [next_config]=next_state(current_config,speed,dt,max_speed)
%config is in order-3 chassis,5 arm and 4 wheel
%speed is in order 5 arm,4 wheel
for i=1:9
%     This loop ensures that speeds are within the limit,if they are
%     not,the respective speeds will be converted into the corresponding
%     max_speed
    if speed(1,i)>max_speed
        if speed(1,i)<0
        speed(1,i)=-max_speed;
        else
        speed(1,i)=max_speed;
        end
    end
end
% The below calculation is odometry and simple first euler   
l=0.47/2;
r=0.0475;
w=0.3/2;
F=r/4*[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w),
    1,1,1,1,
    -1,1,-1,1];
V=F*[speed(1,6);speed(1,7);speed(1,8);speed(1,9)]*dt;
next_config(1,1:3)=V'+current_config(1,1:3);
for i=4:12
    next_config(1,i)=current_config(1,i)+speed(i-3)*dt;
end
    