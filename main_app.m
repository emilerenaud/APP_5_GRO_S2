clear
close all

%----- Angles joints -----%
theta1 = 0;
theta2 = -0.3;
theta3 = 0;
theta4 = 0;
theta5 = 0.5;
theta6 = -1.6;

% Position Defauts
Df1_v_Df1Wv=[0.153758,0.039379,-0.020575]';
Df2_v_Df2Wv=[0.145698,0.079138,-0.039398]';
Df3_v_Df3Wv=[0.153932,0.038521,0.009411]';
Df4_v_Df4Wv=[0.152097,0.047573,0.035692]';
Df5_v_Df5Wv=[0.146104,0.077134,0.030571]';

% Position Pente
Ti1_v_Ti1wV = [0.158920,0.013914,0.028686]';
Ti2_v_Ti2wV = [0.157470,0.021067,0.008891]';
Ti3_v_Ti3wV = [0.153781,0.039266,-0.040587]';
Ti4_v_Ti4wV = [0.152420,0.04597,-0.060395]';
Ti5_v_Ti5wV = [0.150931,0.053326,-0.080185]';

% Dimensions pièces
hg = 0.1;
hd = 0.05;
lb = 0.15;

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
    
pRw = inv(wRa * aRb * bRc * cRd * dRe * eRt * tRp);

%----- Vecteurs / Section du bras -----%
v_w_AwW = [0,0.15,0]';
v_w_BwW = wRa * [0.05,0.1,0]' + v_w_AwW;
v_w_CwW = wRa * aRb * [0,0.5,0]' + v_w_BwW;
v_w_DwW = wRa * aRb * bRc * [0.1,0.02,0]' + v_w_CwW;
v_w_EwW = wRa * aRb * bRc * cRd * [0.3,0,0]' + v_w_DwW;
v_w_TwW = wRa * aRb * bRc * cRd * dRe * [0.02,0,0]' + v_w_EwW;

% point P
v_w_PwW = wRa * aRb * bRc * cRd * dRe * eRt * find_v_t_PwT() + v_w_TwW; % changer le nom de fonction
disp(['Point T en fonction de W en base W : X:', num2str(v_w_TwW(1)), ' Y:', num2str(v_w_TwW(2)), ' Z:', num2str(v_w_TwW(3))]);
% Mettre tous les points dans une matrice pour l'afficher
affichageJoints = [[0,0,0]',v_w_AwW, v_w_BwW,v_w_CwW, v_w_DwW,v_w_EwW,v_w_TwW];
figure(1)
plot3(affichageJoints(1,1:end),affichageJoints(2,1:end),affichageJoints(3,1:end),'k.-','Markersize',15);
grid on;
title('Bras robotisé en fonction des angles des joints'); % Titre
xlabel('x(m)');
ylabel('y(m)');
zlabel('z(m)');
%----- Défaut sur la pièce -----%
0
% Mettre les defauts par rapport W en base W
v_w_D1wW = wRv * Df1_v_Df1Wv + [0.8,0.7,0]'; % + [0.8,0.7,0] c'est l'offset de la camera
v_w_D2wW = wRv * Df2_v_Df2Wv + [0.8,0.7,0]';
v_w_D3wW = wRv * Df3_v_Df3Wv + [0.8,0.7,0]';
v_w_D4wW = wRv * Df4_v_Df4Wv + [0.8,0.7,0]';
v_w_D5wW = wRv * Df5_v_Df5Wv + [0.8,0.7,0]';
% Mettre les defauts par rapport P en base W. Trouver l'offset entre P et D
v_w_D1wP = v_w_D1wW - v_w_PwW;
v_w_D2wP = v_w_D2wW - v_w_PwW;
v_w_D3wP = v_w_D3wW - v_w_PwW;
v_w_D4wP = v_w_D4wW - v_w_PwW;
v_w_D5wP = v_w_D5wW - v_w_PwW;
% Mettre les defauts par rapport a P et base P
v_p_D1wP =  pRw * v_w_D1wP;
v_p_D2wP =  pRw * v_w_D2wP;
v_p_D3wP =  pRw * v_w_D3wP;
v_p_D4wP =  pRw * v_w_D4wP;
v_p_D5wP =  pRw * v_w_D5wP;

MatriceDefauts = [v_p_D1wP,v_p_D2wP,v_p_D3wP,v_p_D4wP,v_p_D5wP];


%----- Points tranche -----%
% Mettre les points en base W
v_w_Ti1wW = wRv * Ti1_v_Ti1wV + [0.8,0.7,0]';
v_w_Ti2wW = wRv * Ti2_v_Ti2wV + [0.8,0.7,0]';
v_w_Ti3wW = wRv * Ti3_v_Ti3wV + [0.8,0.7,0]';
v_w_Ti4wW = wRv * Ti4_v_Ti4wV + [0.8,0.7,0]';
v_w_Ti5wW = wRv * Ti5_v_Ti5wV + [0.8,0.7,0]';
% Trouver offset entre P et Ti en base W
v_w_T1wP = v_w_Ti1wW - v_w_PwW;
v_w_T2wP = v_w_Ti2wW - v_w_PwW;
v_w_T3wP = v_w_Ti3wW - v_w_PwW;
v_w_T4wP = v_w_Ti4wW - v_w_PwW;
v_w_T5wP = v_w_Ti5wW - v_w_PwW;
% Mettre le vecteur en base P
v_p_T1wP = pRw * v_w_T1wP;
v_p_T2wP = pRw * v_w_T2wP;
v_p_T3wP = pRw * v_w_T3wP;
v_p_T4wP = pRw * v_w_T4wP;
v_p_T5wP = pRw * v_w_T5wP;
% Stocker les points dans une matrice
matricePointTranche = [v_p_T1wP,v_p_T2wP,v_p_T3wP,v_p_T4wP,v_p_T5wP];
pointTranche_X = [matricePointTranche(1,1:end)',ones(1,5)'];
pointTranche_Y = matricePointTranche(2,1:end)';

% Trouver l'angle de la tranche
xb = inv(pointTranche_X' * pointTranche_X) * pointTranche_X' * pointTranche_Y;
theta_tranche = abs(atand(xb(1))); % mettre en abs pour avoir le positif

% trouver angle nominal 
theta_nom = atand((hg-hd)/lb);
difference_theta_tranche = theta_tranche - theta_nom

% Calculer la hauteur
difference_hg = abs(hg - xb(2))

% Calculer inégalité
figure(2);
zone_critique = @(x,y) 45*x.^2 + 30*x.*y + 85*y.^2 - 10.8*x - 8.4*y + 0.684;
fcontour(zone_critique,'LineWidth',1,'LineColor','r','LevelList',0);
axis([-0.02 0.16 -0.05 0.16]);
grid on;
hold on;

% Verifier si les points sont dans la zone
xd = MatriceDefauts(1,1:end);
yd = MatriceDefauts(2,1:end);
result = 45*xd.^2 + 30*xd.*yd + 85*yd.^2 - 10.8*xd - 8.4*yd + 0.684;
if result(1) < 0
    disp('Défaut 1 dans la zone');
elseif result(2) < 0
    disp('Défaut 2 dans la zone');
elseif result(3) < 0
    disp('Défaut 3 dans la zone');
elseif result(4) < 0
    disp('Défaut 4 dans la zone');
elseif result(5) < 0
    disp('point 5 dans la zone');
end

% Plot la forme
figure(2)
point_piece = [0,0,lb,lb,0;
               0,hg,hd,0,0];
plot(point_piece(1,1:end),point_piece(2,1:end),'k.-','Markersize',15);
hold on;

% Plot les defauts
figure(2)
plot(v_p_D1wP(1),v_p_D1wP(2),'mx','Markersize',6,'LineWidth',2);
plot(v_p_D2wP(1),v_p_D2wP(2),'cx','Markersize',6,'LineWidth',2);
plot(v_p_D3wP(1),v_p_D3wP(2),'yx','Markersize',6,'LineWidth',2);
plot(v_p_D4wP(1),v_p_D4wP(2),'gx','Markersize',6,'LineWidth',2);
plot(v_p_D5wP(1),v_p_D5wP(2),'bx','Markersize',6,'LineWidth',2);

% Info graphique
xlabel('X (m)');
ylabel('Y (m)');
title('Contour de la pièce avec les défauts et la zone critique') % Titre
legend('Zone critique', 'Forme pièce', 'Défaut 1', 'Défaut 2', 'Défaut 3', 'Défaut 4', 'Défaut 5');
hold on;


%----- Cinématique différentielle -----%
theta_sit_1 = [-0.4,-1.2,0,0,-0.3708,0]';
vitesse_angulaire_Situation_1 = cinematique_diff(theta_sit_1)

theta_sit_2 = [0,0,1.521,0,0,0]';
vitesse_angulaire_Situation_2 = cinematique_diff(theta_sit_2)

% situation C pas impossible, moyen optimiser pour que ça marche environ






