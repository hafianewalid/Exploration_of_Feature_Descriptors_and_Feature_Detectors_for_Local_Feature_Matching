function [P,Hw0,K,s] = calibrate(I)
%% Désignation des coins du rectangle
[BW,corners(1,:),corners(2,:)] = roipoly(I);
%% Calcul des points de fuite v et w
[v, w] = vanishpoints(corners);
%% Centrage de v et w et calcul de la focale (cf. cours)
center = [size(I,1)/2;size(I,2)/2];
v = v - center;
w = w - center;
fsqr = -(v(1)*w(1)+v(2)*w(2)); % cf. cours.
if fsqr < 0
    disp('Calibration incorrecte');
    f = -1;
else
    f = sqrt(fsqr)
end
K = [f 0 center(1); 0 f center(2); 0 0 1];
%% Calcul de l'homographie carré z=0 -> caméra
rect = [0 1 1 0; 0 0 1 1];
H = homography(rect, corners);
%% Calcul de la matrice de projection et du rapport s à partir de l'homographie
[P,s] = H2P(H,K);
Hw0=H*diag([1 1/s 1]);
