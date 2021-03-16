function [v_t_PwT] = find_v_t_PwT()

%----- Angles joints -----%
theta1 = -0.4;
theta2 = -1.2;
theta3 = 0;
theta4 = 0;
theta5 = -0.3708;
theta6 = 0;


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
    
tRp = [ 0,              0,             1;
        1,              0,             0;
        0,              1,             1];

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
% 
v_w_PwT = [0.5994,0,0.1991]' - v_w_TwW;

v_t_PwT = inv(eRt) * inv(dRe) * inv(cRd) * inv(bRc) * inv(aRb) * inv(wRa) * v_w_PwT;