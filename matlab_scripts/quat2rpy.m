% Returns Roll-Pich-Yaw (named bank-attitude-heading)
function [roll,pitch,yaw] = quat2rpy (qx,qy,qz,qw)

yaw = zeros(size(qx));
pitch = zeros(size(qx));
roll = zeros(size(qx));

%yaw(1) = 0

for i=1:size(qx,1),
    yaw(i) = atan2(2*qy(i)*qw(i)-2*qx(i)*qz(i) , 1 - 2*qy(i)^2 - 2*qz(i)^2);
    pitch(i) = asin(2*qx(i)*qy(i) + 2*qz(i)*qw(i));
    roll(i) = atan2(2*qx(i)*qw(i)-2*qy(i)*qz(i) , 1 - 2*qx(i)^2 - 2*qz(i)^2);
end



%except when qx*qy + qz*qw = 0.5 (north pole)
%which gives:
%heading = 2 * atan2(x,w)
%bank = 0
%and when qx*qy + qz*qw = -0.5 (south pole)
%which gives:
%heading = -2 * atan2(x,w)
%bank = 0
%end 