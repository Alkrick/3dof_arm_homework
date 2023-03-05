
syms x0 y0 z0 real

p0 = [x0 y0 z0];
dy = 0.5;

LetB = [0 0      0;
        0 0      dy;
        0 dy/2   dy;
        0 dy/2   dy/2;
        0 0      dy/2;
        0 2*dy/3 dy/2;
        0 2*dy/3 0;
        0 0      0]+p0; 
    
LetI = [0 dy/4   dy;
        0 3*dy/4 dy;
        0 dy/2   dy;
        0 dy/2   0.0;
        0 dy/4   0.0;
        0 3*dy/4 0.0]+p0; 
    
LetI = flipud(LetI); 


LetT = [0 0      dy;
        0 dy     dy;
        0 dy/2   dy;
        0 dy/2   0.0]+p0;
    
    
x0 = 1;
y0 = -0.75;
z0 = 0;

p = subs(LetB);

y0 = y0 + dy;

p = [p;subs(LetI)];

y0 = y0 + dy;

word = double([p;subs(LetT)])';
