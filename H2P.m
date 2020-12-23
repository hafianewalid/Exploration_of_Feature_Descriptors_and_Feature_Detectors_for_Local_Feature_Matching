%cat
function [P,s]=H2P(H,K)
Rt = inv(K)*H;
n = norm(Rt(:,1));
Rt = Rt/n;

H1=Rt(:,1);
H2=Rt(:,2);
H3=Rt(:,3);

s=norm(H2)/norm(H1);
r1=H1;
r2=H2/s;
t=H3;
r3=cross(r1,r2);

P = K*[r1 r2 r3 t];
P = P/P(3,4);
end