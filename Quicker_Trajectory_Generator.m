function [Traj_list,traj,Gripper_state]=Quicker_Trajectory_Generator(Tsei,Tsci,Tscf,Tceg,Tces)
% The function takes the input as specified in the milestone
% We obtain the configurations in the space frame with the following configurations
% Tsegi stands for end effector configuration while grasping initially in
% space frame.Similarly Tsegf is defined.Tsesi,Tsesf corresponds to the stand off configurations 
Tsegi=Tsci*Tceg;
Tsegf=Tscf*Tceg;
Tsesi=Tsci*Tces;
Tsesf=Tscf*Tces;
Traj_list=zeros(2000,13);%initializing the list of configurations
% In each of the calculations below,Screw Trajectory function is used to
% obtain the trajectory.Then a function matrix_to_vector is used for
% conveinence and the matrices are are stored in Traj_list.
% Number of configurations have been adjusted as per the requirement of the
% configurations.
traj1 = CartesianTrajectory(Tsei,Tsesi,5,500,3);
for i=1:500
Traj_list(i,:)=[matrix_to_vector(traj1{i}),0];%here 0 corresponds to gripper state
Traj_list1(i,:)=[matrix_to_vector(traj1{i}),0];
end
traj2 =CartesianTrajectory(Tsesi,Tsegi,2,200,3);
for i=1:200
Traj_list(i+500,:)=[matrix_to_vector(traj2{i}),0];
end
traj3 = CartesianTrajectory(Tsegi,Tsegi,1,100,3);
for i=1:100
Traj_list(700+i,:)=[matrix_to_vector(traj3{i}),1];
traj4 = CartesianTrajectory(Tsegi,Tsesi,2,200,3);
end
for i=1:200
Traj_list(i+800,:)=[matrix_to_vector(traj4{i}),1];
end
traj5 = CartesianTrajectory(Tsesi,Tsesf,4,400,3);
for i=1:400
Traj_list(i+1000,:)=[matrix_to_vector(traj5{i}),1];
end
traj6 = CartesianTrajectory(Tsesf,Tsegf,2,200,3);
for i=1:200
Traj_list(i+1400,:)=[matrix_to_vector(traj6{i}),1];
end
traj7 = CartesianTrajectory(Tsegf,Tsegf,2,200,3);
for i=1:100
Traj_list(1600+i,:)=[matrix_to_vector(traj7{i}),0];
traj8 =CartesianTrajectory(Tsegf,Tsesf,2,200,3);
end
for i=1:200
Traj_list(i+1800,:)=[matrix_to_vector(traj8{i}),0];
end
Gripper_state=Traj_list(1:1999,13);
csvwrite('q_Traj_list.csv',Traj_list);
csvwrite('q_Traj_list1.csv',Traj_list1);%We write the configurations in a csv file
traj=[traj1,traj2,traj3,traj4,traj5,traj6,traj7,traj8];%traj is a list of trajectory matrices in a cell.It is used in future calculations 
