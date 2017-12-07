A1 = -(4*a(2)*y0+2*a(3));
A2 = 2*a(3)*x0-2*a(1)*y0;
A3 = 4*a(4)-4*a(2);
A4 = -4*a(4)*x0-2*a(1);

G4 = a(2)*A3^2;
G3 = a(1)*A3^2 + 2*a(2)*A3*A4;
G2 = a(2)*A4^2 + 2*a(1)*A3*A4 + a(3)*A1*A3 + a(4)*A1^2 + A3^2;
G1 = a(1)*A4^2 + a(3)*A1*A4 + a(3)*A2*A3 + 2*a(4)*A1*A2 + 2*A3*A4;
G0 = a(4)*A2^2 + a(3)*A2*A4 + A4^2;

opx = x0;
for opi = 1:5
    opx = opx - (G4*opx^4+G3*opx^3+G2*opx^2+G1*opx+G0)/(4*G4*opx^3+3*G3*opx^2+2*G2*opx^1+G1);
end
opy = (A1*opx+A2)/(A3*opx+A4);