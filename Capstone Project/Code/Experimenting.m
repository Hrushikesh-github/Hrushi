function  Experimenting(Tsci,Tscf,Config,Kp,Ki,dt,max_speed)
% The function takes cube's initial and final SE(3) representation and
% takes a row vector 'Config' which is the initial configuration of the
% you-bot and the order is similar to that mentioned in next_state function
%Few constants are defined RTse is the initial reference configuration as
%specified in the project.
RTse=[0,0,1,0;0,1,0,0;-1,0,0,0.5;0,0,0,1];
Tceg=[0,0,1,0;0,1,0,0;-1,0,0,0;0,0,0,1];
Tces=[0,0,1,0;0,1,0,0;-1,0,0,0.06;0,0,0,1];
M = [1,0,0,0.033;0,1,0,0;0,0,1,0.6546;0,0,0,1];
Blist = [0,0,0,0,0;0,-1,-1,-1,0;1,0,0,0,1;0,-0.5076,-0.3526,-0.2176,0;0.033,0,0,0,0;0,0,0,0,0];
Tbo = [1,0,0,0.1662;0,1,0,0;0,0,1,0.0026;0,0,0,1];
Tsb = [cos(Config(1,1)),-sin(Config(1,1)),0,Config(1,2);sin(Config(1,1)),cos(Config(1,1)),0,Config(1,3);0,0,1,0.0963;0,0,0,1];
alist = Config(1,4:8)';
Toe = FKinBody(M,Blist,alist);
Tse = Tsb*Tbo*Toe;%Tse which is end effector's initial configuration is used in trajectory generation.
[Traj_list,traj,Gripper_state]=Quicker_Trajectory_Generator(RTse,Tsci,Tscf,Tceg,Tces);%Trajectory Generated!
% Few constants are defined which will be used in feedback control and odometry
integral_error=0;
l=0.47/2;
r=0.0475;
w=0.3/2;
F=r/4*[-1/(l+w),1/(l+w),1/(l+w),-1/(l+w),
    1,1,1,1,
    -1,1,-1,1];
F6 = [0 0 0 0;0 0 0 0;F;0 0 0 0];
% The below for loop calculates and writes the list of configurations
% followed by the end effector.Also the error twist Xerr is also calculated and written in a csv file 
for i=1:1999
    Jarm = JacobianBody(Blist,alist);
    Jbase = Adjoint(inv(Toe)*inv(Tbo))*F6;
    Je = [Jbase Jarm]; 
    [V,Xerr,integral_error]=feedback_control(Tse,traj{i},traj{i+1},Kp,Ki,dt,integral_error);%The end-effector twist is calculated using the feedbact_control funtion
     u= pinv(Je,0.0001)*V;%As we already obtained the jacobian we can find the speed which the wheels and joint arms must run through to obtain the end effector twist  
%      The below for loop is stored in a row matrix 'speed' so that it can
%      used conveniently to run the next_state function
     for j=1:5
     speed(1,j)=u(4+j,1);
     end
     for k=1:4
     speed(1,k+5)=u(k,1);
     end
    [next_config]=next_state(Config,speed,dt,max_speed);%the next configuration is calculated
    [next_config]=Test_Joint_Limits(next_config,Config,Je,V,dt,max_speed);%Joint angles are tested,ensuring that they are within the specified limits  
    Config=next_config;%Modifying the Config variable for the next cycle of the loop
    Trajectories(i,:)=Config;
    alist = Config(1,4:8)';
    Toe = FKinBody(M,Blist,alist);
    Tsb = [cos(Config(1,1)),-sin(Config(1,1)),0,Config(1,2);sin(Config(1,1)),cos(Config(1,1)),0,Config(1,3);0,0,1,0.0963;0,0,0,1];
    Tse = Tsb*Tbo*Toe;%The end effector configuration is modified and is ready to be used in next cycle of loop
    X_errlist(i,:) = Xerr;%Stores the Xerr matrix in a bigger matrix which already has the previous Xerr matrices stored 
end
Trajectories=[Trajectories,Gripper_state];
plot(X_errlist);
legend('Xerr1','Xerr2','Xerr3','Xerr4','Xerr5','Xerr6');
csvwrite('Besty.csv',Trajectories);
csvwrite('Xerr_Best.csv',X_errlist);