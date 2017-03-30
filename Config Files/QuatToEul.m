function Eul = QuatToEul(Quat)

q0 = Quat(1);
q1 = Quat(2);
q2 = Quat(3);
q3 = Quat(4);

Eul = zeros(1,3);

Eul(1) = atan2(2*(q0*q1 + q2*q3), (1-2*(q1^2 + q2^2)));
Eul(2) = asin(2*(q0*q2 - q1*q3));
Eul(3) = atan2(2*(q0*q3 + q1*q2), (1-2*(q2^2 + q3^2)));

