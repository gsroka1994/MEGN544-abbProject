% createLink
%
% L = createLink(a, d, alpha, theta, offset, centerOfMass, mass, interita, isRotary) 
%     Creates a link structure with the inpur arguments as data memebrs.
%     All vectors and tensors are expressed in the link coordinate's frame.
%     When called, pass an empty array [] to the joint variable that
%     changes (i.e theta = [] for rotary, d = [] for prismatic)
% 
% L = Link
% A Matlab structure composed of the following elements:
% 
% a = the distance in the X direction
% d = the distance in the Z direction
% alpha = the angle in the X direction
% theta = the angle in the Z direction
% offset = The number of radians different between encoder orientation and
% DH-zero-angle offset
% com = Position of COM
% mass = mass of link
% inertia = inertia of link
% isRotary = Boolean true f rotary, false if prismatic
% 
% Gunnar Sroka
% 10649539
% MEGN544 
% 10/4/2020
function L = createLink(a, d, alpha, theta, offset, centerOfMass, mass, inertia)
  L.a = a;
  L.d = d;
  L.alpha = alpha;
  L.theta = theta;
  L.offset = offset;
  L.com = centerOfMass;
  L.mass = mass;
  L.inertia = inertia;
  
  if isempty(theta)
      L.isRotary = 1;
  elseif isempty(d)
      L.isRotary = 0;
  else
      L.isRotary = -1;
  end
  
end

