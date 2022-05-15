function [distance] = pointPlaneDistance(planeNormal, pointOnPlane, P)
%POINTPLANEDISTANCE Calculates the perpendicular distance from a point to
%the specified plane
a = planeNormal(1);
b = planeNormal(2);
c = planeNormal(3);
d = -(a*pointOnPlane(1)+b*pointOnPlane(2)+c*pointOnPlane(3));

distance = (abs(a*P(1)+b*P(2)+c*P(3) + d)) / (sqrt(a^2+b^2+c^2));
end

