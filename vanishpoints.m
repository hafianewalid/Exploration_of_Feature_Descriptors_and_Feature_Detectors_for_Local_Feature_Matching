%cat
function [v, w] = vanishpoints(corners)
p1 = [corners(:,1);1];
p2 = [corners(:,2);1];
p3 = [corners(:,3);1];
p4 = [corners(:,4);1];

v1=cross(p1,p2);
v2=cross(p3,p4);
ch=cross(v1,v2);
v=ch(1:2)/ch(3);


v1=cross(p1,p4);
v2=cross(p2,p3);
ch=cross(v1,v2);
w=ch(1:2)/ch(3);


for i = 1:4
    line([v(1);corners(1,i)],[v(2);corners(2,i)]);
    line([w(1);corners(1,i)],[w(2);corners(2,i)]);
end