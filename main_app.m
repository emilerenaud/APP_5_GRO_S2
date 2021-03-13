%----- Angles joints -----%
angle1 = 0;
angle2 = 0;
angle3 = 0;
angle4 = 0;
angle5 = 0;
angle6 = 0;

%----- Matrice de rotation -----%
% Joint #1
wRa = [ cos(angle1),    0,  sin(angle1);
        0,              1,  0;
        -sin(angle1),   0,  cos(angle1)];
    
% Joint #2
aRb = [ cos(angle2),    -sin(angle2),   0;
        sin(angle2),    cos(angle2),    0;
        0,              0,              1];
    
% Joint #3
bRc = [ cos(angle3),    -sin(angle3),   0;
        sin(angle3),    cos(angle3),    0;
        0,              0,              1];
    
% Joint #4
cRd = [ 1,              0,              0;
        0,              cos(angle4),    -sin(angle4);
        0,              sin(angle4),    cos(angle4)];
    
% Joint #5
dRe = [ cos(angle5),    -sin(angle5),   0;
        sin(angle5),    cos(angle5),    0;
        0,              0,              1];
    
% Joint #6
eRt = [ 1,              0,              0;
        0,              cos(angle6),    -sin(angle6);
        0,              sin(angle6),    cos(angle6)];
   
% Camera
wRv = [ -1,             0,              0;
        0,              -1,             0;
        0,              0,              1];
    
    
%----- Vecteurs / Section du bras -----%
v_a_w = wRa * [0,0.15,0]';
v_b_a = wRa * aRb * [0.05,0.1,0]';



