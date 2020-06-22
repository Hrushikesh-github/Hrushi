Hi! 
For Reviewer:

I have written my code for the Capstone Project in MATLAB. I have written and used six funtions for the project,they are
matrix_to_vector;next_state;feedback_control;Trajectory_Generator;Control;Test_Joint_Limits.
I suggest the reviewer to look into the functions in the given order:next_state,Trajectory_Generator,matrix_to_vector,feedback_control,Control,Test_Joint_Limits.

The function Test_Joint_Limit was written as mentioned in the wiki, but with a caveat which is mentioned in the function.
I have implemented the singularity allowance using the tolerance option for pseudo inverse(i.e in the form pinv(Je,0.0001))

Related to Capstone Project:
 There are certain issues that I believe I did not understand completely such as Singularity,various responses to 
different values of Kp and Ki.But I am sure I will understand them when I revise the topics once again.

Have a good day!
