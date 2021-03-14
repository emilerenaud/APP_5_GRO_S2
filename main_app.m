clear
close
%----- Angles joints -----%
theta1 = 0;
theta2 = -0.3;
theta3 = 0;
theta4 = 0;
theta5 = 0.5;
theta6 = -1.6;

Df1_c_df1Wc=[0.153758,0.039379,-0.020575]';
Df2_c_Df2Wc=[0.145698,0.079138,-0.039398]';
Df3_c_Df3Wc=[0.153932,0.038521,0.009411]';
Df4_c_Df4Wc=[0.152097,0.047573,0.035692]';
Df5_c_Df5Wc=[0.146104,0.077134,0.030571]';

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
v_w_TwW = wRa * aRb * bRc * cRd * dRe * [0.02,0,0]' + v_w_EwW
% 
difference = [0.5994,0,0.1991]' - v_w_TwW;
% Grosse matrice en colonnes
%v_w_PwW = wRa * aRe * bRc * cRd * dRe * eRt * difference + v_w_TwW;
v_w_PwW = wRa * aRb * bRc * cRd * dRe * positionEffecteur() + v_w_TwW % changer le nom de fonction
% Mettre tous les points dans une matrice pour l'afficher
affichagePoint = [[0,0,0]',v_w_AwW, v_w_BwW,v_w_CwW, v_w_DwW,v_w_EwW,v_w_TwW,v_w_PwW];

plot3(affichagePoint(1,1:end),affichagePoint(2,1:end),affichagePoint(3,1:end))
grid on

v_w_DwW = wRv * Df1_c_df1Wc + [0.8,0.7,0]' % on pense que c'est bon

v_w_DwP = v_w_DwW - v_w_PwW;

v_p_DwP =  inv(eRt) * inv(dRe) * inv(cRd) * inv(bRc) * inv(aRb) * inv(wRa) * inv(tRp) * v_w_DwP









