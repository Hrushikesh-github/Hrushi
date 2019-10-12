function [V,Xerr,integral_error]=feedback_control(X,Xd,Xdn,Kp,Ki,dt,integral_error)
% This configuration takes current configuration,current and the next reference
% configuration.Also the Kp,Ki values,dt(it is 0.01 always) and the
% integral_error which is added up each time this function runs.Initially
% we put integral_error=0,for the first time we are using it.
Xerr = se3ToVec(logm(inv(X)*Xd));%Xerr is the error twist matrix
Vd = se3ToVec((1/dt)*logm(inv(Xd)*Xdn));
integral_error = integral_error+(Ki*Xerr*dt);
V = (Adjoint(inv(X)*Xd)*Vd)+(Kp*Xerr)+integral_error;
end