function[cmdV,cmdW] = limitCmds(fwdVel,angVel,maxV,wheel2Center)
% LIMITCMDS: scale forward and angular velocity commands to avoid
% saturating motors.
% 
%   [CMDV,CMDW] = LIMITCMDS(FWDVEL,ANGVEL,MAXV,WHEEL2CENTER) returns the 
%   forward and angular velocity commands to drive a differential-drive 
%   robot whose wheel motors saturate at +/- maxV.
% 
%   INPUTS
%       fwdVel      desired forward velocity (m/s)
%       angVel      desired angular velocity (rad/s)
%       maxV        maximum motor velocity (assumes symmetric saturation)
%       wheel2Center distance from the wheels to the center of the robot(in meters)
% 
%   OUTPUTS
%       cmdV        scaled forward velocity command (m/s)
%       cmdW        scaled angular velocity command (rad/s)
% 
% 
%   Cornell University
%   Autonomous Mobile Robots
%   Homework #1
%   LASTNAME, FIRSTNAME 
Vr = angVel*wheel2Center + fwdVel; Vl = fwdVel - angVel*wheel2Center;
if abs(angVel)>= 1e-5
    R = wheel2Center*(Vr + Vl)/(Vr - Vl);
    if abs(Vr) > maxV || abs(Vl) > maxV
        if abs(Vr) >= abs(Vl)
            if Vr>0
                Vr = maxV; Vl = Vr*(R-wheel2Center)/(R+wheel2Center);
            else
                Vr = -maxV; Vl = Vr*(R-wheel2Center)/(R+wheel2Center);
            end
        else
            if Vl>0
                Vl = maxV; Vr = Vl*(R+wheel2Center)/(R-wheel2Center);
            else
                Vl = -maxV; Vr = Vl*(R+wheel2Center)/(R-wheel2Center);
            end
        end
    end
    cmdW = (Vr - Vl)/2/wheel2Center;
    cmdV = R*cmdW; 
else
    if abs(Vr) > maxV
       Vr = maxV*Vr/abs(Vr);  Vl = Vr;
    end
    cmdV = Vr; cmdW = 0;
end
