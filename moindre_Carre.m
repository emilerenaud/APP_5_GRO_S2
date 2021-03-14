%calcul moindre carre
% x = (At * A )-1 * At * y
hg = 0.1;
hd = 0.05;
lb = 0.15;

Y = [0.013914,0.021067,0.039266,0.04597,0.053326]';
Z = [0.028686,0.008891,-0.040587,-0.060395,-0.080185,
     1,       1,        1,        1,        1]';

plot(Z,Y)

%X1 = A' * inv(A*A') * Y
X2 = inv(Z' * Z) * Z' * Y
%X3 = A\Y
 % x et X2 donne la meme chose.
%atand(X1)
atand(X2)
% trouver angle nominal 
theta_nom = atand((hg-hd)/lb)

diffenceAngle = abs(atand(X2(1))) - theta_nom

% valider hauteur hg avec l'angle trouvé
hg2 = tand(abs(atand(X2(1)))) * 0.15 + 0.05
differenceHg = hg2 - hg

%atand(X3)
