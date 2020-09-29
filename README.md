## Mobile Manipulation of youBot
youBot is Mobile 5R robot on an omniwheel robot, I have written few programs in MATLAB to manipulate the mobile manipulator using the V-REP simulator.
This is a capstone project in the specialization [Modern Robotics, Motion Planning and Control](https://www.coursera.org/specializations/modernrobotics)

<img width="400" alt="youbot" src="https://user-images.githubusercontent.com/56476887/94597342-192d6280-02ab-11eb-9de7-051eff086b6b.png">

![axes](https://user-images.githubusercontent.com/56476887/94597633-6f020a80-02ab-11eb-96db-8c171d258461.png)

The 'control' code controls the mobile robot and it uses functions written in files named 'next state','trajectory generator',
'feedback contol' and other functions. Detailed description of each is mentioned in respective files.

I suggest to anyone interested to understand the codes to look into the functions in the given order:
next_state,Trajectory_Generator,matrix_to_vector,feedback_control,Control,Test_Joint_Limits.



## Results:

Videos of my simulation can be found [here](https://drive.google.com/drive/folders/1X04_qlfhq0yZ_v_NTAelq3jnIdfCnfK-?usp=sharing)

# The joint values error from the desired trajectory for different PID values

![newTask](https://user-images.githubusercontent.com/56476887/85314929-cf78f300-b4d7-11ea-8ba9-5b6c2aea2bf1.png)

Overshoot because of the values of PID
![overshoot](https://user-images.githubusercontent.com/56476887/85314951-d6a00100-b4d7-11ea-98ad-664ee8ebd6ef.png)


### The best achieved result 
![Best](https://user-images.githubusercontent.com/56476887/85314963-dbfd4b80-b4d7-11ea-9ec5-7158e50d3657.png)


