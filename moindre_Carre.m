%calcul moindre carre
% x = (At * A )-1 * At * y

Y = [0.013914,0.021067,0.039266,0.04597,0.053326]';
A = [0.028686,0.008891,-0.040587,-0.060395,-0.080185]';

x = inv(A'*A) * A' * Y;

X2 = A\Y
 % x et X2 donne la meme chose.
 
atand(x/1)