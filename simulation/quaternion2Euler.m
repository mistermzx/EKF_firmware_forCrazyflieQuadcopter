function RPY = quaternion2Euler(q)
%quaternions to euler angles: q = [q0,q1,q2,q3]
% RPY = [roll, pitch, yaw]
y = atan2(2*q(2)*q(3) + 2*q(1)*q(4), q(2)*q(2) + q(1)*q(1)- q(4)*q(4) - q(3)*q(3));
p = -asin(2.0*q(2)*q(4) - 2.0*q(1)*q(3));
r = atan2(2.0*q(3)*q(4) + 2.0*q(1)*q(2), q(4)*q(4) - q(3)*q(3)- q(2)*q(2) + q(1)*q(1));
RPY = [r,p,y];
end

