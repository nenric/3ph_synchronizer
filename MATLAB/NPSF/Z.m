syms wn s zeta Ts z
y= wn^2/ ( s^2+(2*zeta*wn*s)+wn^2 );
s2z=2*(1-1/z)/(Ts*(1+1/z));     % Bilinear Transform, â€˜sâ€™ To â€˜zâ€™
y = subs(y, s, s2z);            % Convert Continuous To Discrete
[yn,yd] = numden(y);            % Numerator, Denominator
yn = collect(yn, z)             % Numerator Polynomial In â€˜zâ€™
yd = collect(yd, z)             % Denominator Polynomial In â€˜zâ€™
yncf = coeffs(yn, z)            % Numerator Coefficients
ydcf = coeffs(yd, z)            % Denominator Coefficients

% Extraer coeficientes en orden
yncf = coeffs(yn, z, 'All');
ydcf = coeffs(yd, z, 'All');

b0 = yncf(1);
b1 = yncf(2);
b2 = yncf(3);

a0 = ydcf(1);
a1 = ydcf(2);
a2 = ydcf(3);

% Normalizar todos los coeficientes dividiendo por a0
b0_n = b0 / a0;
b1_n = b1 / a0;
b2_n = b2 / a0;
a1_n = a1 / a0;
a2_n = a2 / a0;

fprintf('y[k] = (%s)*x[k] + (%s)*x[k-1] + (%s)*x[k-2] - (%s)*y[k-1] - (%s)*y[k-2]\n', ...
    char(b0_n), char(b1_n), char(b2_n), char(a1_n), char(a2_n));


