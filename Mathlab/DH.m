syms a1 a2 a3 a5 d1 d2 d3 t4 d5 a4



m01 = homo(-pi/2, pi/2, 0, d1+a1);
m12 = homo(-pi/2, pi/2, 0, d2+a2);
m23 = homo(0, -pi/2, 0, d3+a3);
m34 = homo(t4, pi/2, 0, a4);
m45 = homo(0, 0, 0, a4+d5);

m03 = m01 * m12 * m23
m35 = m34 * m45
m05 = m03 * m35

