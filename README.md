# Capstone-Project---Robotics-Specilisation
For the capstone project dealing with a 5R robot on an omniwheel robot, I have written few programs to control the mobile manipulator.
This is the video link for the assignment:  https://www.youtube.com/watch?v=Q1CekpBW6Js&feature=youtu.be
Here is the link for the course:  https://www.coursera.org/learn/modernrobotics-course6
The 'control' code controls the mobile robot and it uses functions written in files named 'next state','trajectory generator',
'feedback contol' and other functions. Detailed description of each is mentioned in respective files.

I suggest to anyone interested to understand the codes to look into the functions in the given order:
next_state,Trajectory_Generator,matrix_to_vector,feedback_control,Control,Test_Joint_Limits.

Results:

![newTask](https://user-images.githubusercontent.com/56476887/85314929-cf78f300-b4d7-11ea-8ba9-5b6c2aea2bf1.png)

### Overshoot because of the values of PID
![overshoot](https://user-images.githubusercontent.com/56476887/85314951-d6a00100-b4d7-11ea-98ad-664ee8ebd6ef.png)


### The best achieved result 
![Best](https://user-images.githubusercontent.com/56476887/85314963-dbfd4b80-b4d7-11ea-9ec5-7158e50d3657.png)

