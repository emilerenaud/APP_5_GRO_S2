function [pos_effect, matrice_points] = positionEffecteur(theta1,theta2,theta3,theta4,theta5,theta6)

%----- Matrice de rotation -----%
% Joint #1
wRa = [ cos(theta1),    0,  sin(theta1);
        0,              1,  0;
        -sin(theta1),   0,  cos(theta1)];
    
% Joint #2
aRb = [ cos(theta2),    -sin(theta2),   0;
        sin(theta2),    cos(theta2),    0;
        0,              0,              1];
    
% Joint #3
bRc = [ cos(theta3),    -sin(theta3),   0;
        sin(theta3),    cos(theta3),    0;
        0,              0,              1];
    
% Joint #4
cRd = [ 1,              0,              0;
        0,              cos(theta4),    -sin(theta4);
        0,              sin(theta4),    cos(theta4)];
    
% Joint #5
dRe = [ cos(theta5),    -sin(theta5),   0;
        sin(theta5),    cos(theta5),    0;
        0,              0,              1];
    
% Joint #6
eRt = [ 1,              0,              0;
        0,              cos(theta6),    -sin(theta6);
        0,              sin(theta6),    cos(theta6)];
   
% Camera
wRv = [ -1,             0,              0;
        0,              -1,             0;
        0,              0,              1];
    
    
%----- Vecteurs / Section du bras -----%
v_w_AwW = [0,0.15,0]';
v_w_BwW = wRa * [0.05,0.1,0]' + v_w_AwW;
v_w_CwW = wRa * aRb * [0,0.5,0]' + v_w_BwW;
v_w_DwW = wRa * aRb * bRc * [0.1,0.02,0]' + v_w_CwW;
v_w_EwW = wRa * aRb * bRc * cRd * [0.3,0,0]' + v_w_DwW;
v_w_TwW = wRa * aRb * bRc * cRd * dRe * [0.02,0,0]' + v_w_EwW;

% Grosse matrice en colonnes
affichagePoint = [[0,0,0]',v_w_AwW, v_w_BwW,v_w_CwW, v_w_DwW,v_w_EwW,v_w_TwW];

affichagePoint(1,1:end);
affichagePoint(2,1:end);
affichagePoint(3,1:end);

plot3(affichagePoint(1,1:end),affichagePoint(2,1:end),affichagePoint(3,1:end))
grid on

matrice_points = affichagePoint;
pos_effect = v_w_TwW;