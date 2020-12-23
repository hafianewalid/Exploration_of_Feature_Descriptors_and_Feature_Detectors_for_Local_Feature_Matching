%cat
function [H] = homography(points1, points2)
x1 = points1(1,1);
x2 = points1(1,2);
x3 = points1(1,3);
x4 = points1(1,4);
y1 = points1(2,1);
y2 = points1(2,2);
y3 = points1(2,3);
y4 = points1(2,4);
xp1 = points2(1,1);
xp2 = points2(1,2);
xp3 = points2(1,3);
xp4 = points2(1,4);
yp1 = points2(2,1);
yp2 = points2(2,2);
yp3 = points2(2,3);
yp4 = points2(2,4);
A = [
    x1 y1 1 0 0 0 -xp1*x1 -xp1*y1 -xp1;
    0 0 0 x1 y1 1 -yp1*x1 -yp1*y1 -yp1;
    x2 y2 1 0 0 0 -xp2*x2 -xp2*y2 -xp2;
    0 0 0 x2 y2 1 -yp2*x2 -yp2*y2 -yp2;
    x3 y3 1 0 0 0 -xp3*x3 -xp3*y3 -xp3;
    0 0 0 x3 y3 1 -yp3*x3 -yp3*y3 -yp3;
    x4 y4 1 0 0 0 -xp4*x4 -xp4*y4 -xp4;
    0 0 0 x4 y4 1 -yp4*x4 -yp4*y4 -yp4;
    ];

h = null(A);
H =[h(1:3,1)';h(4:6,1)';h(7:9,1)'];
H = H/H(3,3);
