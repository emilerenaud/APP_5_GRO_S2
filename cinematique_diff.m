% Longueur bras
function [vitesse_angulaire] = cinematique_diff(theta_in)
L1 = 0.15;
L2x = 0.05;
L2y = 0.1;
L3 = 0.5;
L4y = 0.02;
L4x = 0.1;
L5 = 0.3;
% theta_1 = [-0.4,-1.2,0,0,-0.3708,0]';
% theta_2 = [0,0,1.521,0,0,0]';
theta = [0,0,0,0,0,0]';

theta = theta_in;

a1 = -L3 * sin(theta(2)) - L4y * sin(theta(2) + theta(3)) + (L4x + L5) * cos(theta(2) + theta(3));
a2 = -L4y * sin(theta(2) + theta(3)) + (L4x + L5) * cos(theta(2) + theta(3));

j_q_a = [0,a1,a2,0,0,0];

% pour le point 1
vit_angulaire_A = pinv(j_q_a) * 1;

%----- Numéro B -----%
% Longueur des axes avec les angles dans l'annexe
h = L1 + L2y + L3 * cos(theta(2)) + L4y * cos(theta(2) + theta(3)) + (L4x + L5) * sin(theta(2) + theta(3));
r = L2x - L3 * sin(theta(2)) - L4y * sin(theta(2) + theta(3)) + (L4x + L5) * cos(theta(2) + theta(3));

j_q_b = [-r * sin(theta(1)),  -L3 * cos(theta(2)) - L4y * cos(theta(2) + theta(3)) - (L4x + L5) * sin(theta(2) + theta(3)),     -L4y * cos(theta(2) + theta(3)) - (L4x + L5) * sin(theta(2) + theta(3)),     0,     0,     0;
         0,                   -L3 * sin(theta(2)) - L4y * sin(theta(2) + theta(3)) + (L4x + L5) * cos(theta(2) + theta(3)),     -L4y * sin(theta(2) + theta(3)) + (L4x + L5) * cos(theta(2) + theta(3)),     0,     0,     0;
         -r * cos(theta(1)),                                                                                   0,                                                                                       0,     0,     0,     0];

% pour le point 1
% calcul direct
vit_angulaire_B_direct = pinv(j_q_b) * [0,1,0]';

% calcul pseudo-inverse 
vit_angulaire_B = j_q_b' * inv(j_q_b * j_q_b') * [0,1,0]';


% output
vitesse_angulaire = [vit_angulaire_A,vit_angulaire_B];


