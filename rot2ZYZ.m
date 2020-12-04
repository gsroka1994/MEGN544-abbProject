function [phi,theta,psi] = rot2ZYZ(R)
%RotZYZ(phi,theta,psi)=RotZ(phi)RotY(theta)RotZ(psi)
phi=zeros(2,1);
theta=zeros(2,1);
psi=zeros(2,1);

theta(1,1)=atan2((R(1,3)^2+R(2,3)^2)^.5,R(3,3));
theta(2,1)=atan2(-(R(1,3)^2+R(2,3)^2)^.5,R(3,3));

if abs(theta)<=eps
    psi(1,1)=0;
    psi(2,1)=pi;
    
    phi(1,1)=atan2(R(2,1),R(1,1));
    phi(2,1)=atan2(-R(2,1),-R(1,1));
else
    phi(1,1)=atan2(R(2,3)/sin(theta(1,1)),R(1,3)/sin(theta(1,1)));
    phi(2,1)=atan2(R(2,3)/sin(theta(2,1)),R(1,3)/sin(theta(2,1)));

    psi(1,1)=atan2(R(3,2)/sin(theta(1,1)),-R(3,1)/sin(theta(1,1)));
    psi(2,1)=atan2(R(3,2)/sin(theta(2,1)),-R(3,1)/sin(theta(2,1)));
end
end