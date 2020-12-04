% cpMap
%
% X = cpMap(w) Returns the matrix packing of the cross product operator. 
%              Ie. Given vectors W and V, cross(W) * V = W x V
% 
% X = a matrix representaion of the cross product
% 
% w = the vector that the map is being constructed from
% 
% Gunnar Sroka
% 10649539
% MEGN544 
% 9/4/2020

function X = cpMap(w)
  X = [0, -w(3), w(2);
      w(3), 0, -w(1);
      -w(2), w(1), 0];
end

