function T = DH(q, d, alpha, a)

Rz = [cos(q) -sin(q) 0 0; sin(q) cos(q) 0 0; 0 0 1 0; 0 0 0 1];
Td = [1 0 0 0; 0 1 0 0; 0 0 1 d; 0 0 0 1];
Ta = [1 0 0 a; 0 1 0 0; 0 0 1 0; 0 0 0 1];
Ra = [1 0 0 0; 0 cos(alpha) -sin(alpha) 0; 0 sin(alpha) cos(alpha) 0; 0 0 0 1];

T = Rz*Td*Ta*Ra;