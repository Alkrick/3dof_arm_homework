function zdot=NewtonRHS3(t,z,flag,m,I, l, g, tau, fExt)   

q1 = z(1);   u1 = z(4);                         
q2 = z(2);   u2 = z(5);                         
q3 = z(3);   u3 = z(6);                         

m1 = m(1);   m2 = m(2);  m3 = m(3);             
l1 = l(1);   l2 = l(2);  l3 = l(3);             
I1 = I(:,:,1);   I2 = I(:,:,2);  I3 = I(:,:,3); 

fx = fExt(1); fy = fExt(2); fz = fExt(3);        
nx = fExt(4); ny = fExt(5); nz = fExt(6);        

I1_1 = I1(1,1);I1_2 = I1(2,2);I1_3 = I1(3,3);    
I1_12 = I1(1,2);I1_13 = I1(1,3);I1_23 = I1(2,3); 

I2_1 = I2(1,1);I2_2 = I2(2,2);I2_3 = I2(3,3);    
I2_12 = I2(1,2);I2_13 = I2(1,3);I2_23 = I2(2,3); 

I3_1 = I3(1,1);I3_2 = I3(2,2);I3_3 = I3(3,3);    
I3_12 = I3(1,2);I3_13 = I3(1,3);I3_23 = I3(2,3); 

T1 = tau(1); T2 = tau(2); T3 = tau(3);
M11 = (I3_1*cos(2*q2 + 2*q3))/2 - I2_1/2 - I2_2/2 - I3_1/2 - I3_2/2 - I1_3 - (I3_2*cos(2*q2 + 2*q3))/2 - I3_12*sin(2*q2 + 2*q3) - (l2^2*m2)/8 - (l2^2*m3)/2 - (l3^2*m3)/8 + (I2_1*cos(2*q2))/2 - (I2_2*cos(2*q2))/2 - I2_12*sin(2*q2) - (l2^2*m2*cos(2*q2))/8 - (l2^2*m3*cos(2*q2))/2 - (l3^2*m3*cos(2*q2 + 2*q3))/8 - (l2*l3*m3*cos(q3))/2 - (l2*l3*m3*cos(2*q2 + q3))/2; 
M12 = - I3_23*cos(q2 + q3) - I3_13*sin(q2 + q3) - I2_23*cos(q2) - I2_13*sin(q2); 
M13 = - I3_23*cos(q2 + q3) - I3_13*sin(q2 + q3); 

M21 = - I3_23*cos(q2 + q3) - I3_13*sin(q2 + q3) - I2_23*cos(q2) - I2_13*sin(q2); 
M22 = - I2_3 - I3_3 - (l2^2*m2)/4 - l2^2*m3 - (l3^2*m3)/4 - l2*l3*m3*cos(q3); 
M23 = - I3_3 - (l3^2*m3)/4 - (l2*l3*m3*cos(q3))/2; 

M31 = - I3_23*cos(q2 + q3) - I3_13*sin(q2 + q3); 
M32 = - I3_3 - (l3^2*m3)/4 - (l2*l3*m3*cos(q3))/2; 
M33 = - I3_3 - (l3^2*m3)/4; 

RHS1 = I3_13*u2^2*cos(q2 + q3) - fz*l2*cos(q2) - T1 + I3_13*u3^2*cos(q2 + q3) - I3_23*u2^2*sin(q2 + q3) - I3_23*u3^2*sin(q2 + q3) + I2_13*u2^2*cos(q2) - I2_23*u2^2*sin(q2) - fz*l3*cos(q2 + q3) + I3_1*u1*u2*sin(2*q2 + 2*q3) + I3_1*u1*u3*sin(2*q2 + 2*q3) - I3_2*u1*u2*sin(2*q2 + 2*q3) - I3_2*u1*u3*sin(2*q2 + 2*q3) + 2*I3_13*u2*u3*cos(q2 + q3) - 2*I3_23*u2*u3*sin(q2 + q3) + 2*I2_12*u1*u2*cos(2*q2) + I2_1*u1*u2*sin(2*q2) - I2_2*u1*u2*sin(2*q2) + 2*I3_12*u1*u2*cos(2*q2 + 2*q3) + 2*I3_12*u1*u3*cos(2*q2 + 2*q3) - (l2^2*m2*u1*u2*sin(2*q2))/4 - l2^2*m3*u1*u2*sin(2*q2) - (l3^2*m3*u1*u2*sin(2*q2 + 2*q3))/4 - (l3^2*m3*u1*u3*sin(2*q2 + 2*q3))/4 - (l2*l3*m3*u1*u3*sin(q3))/2 - l2*l3*m3*u1*u2*sin(2*q2 + q3) - (l2*l3*m3*u1*u3*sin(2*q2 + q3))/2; 
RHS2 = fy*l3 - T2 + fy*l2*cos(q3) + fx*l2*sin(q3) - I2_12*u1^2*cos(2*q2) - (I2_1*u1^2*sin(2*q2))/2 + (I2_2*u1^2*sin(2*q2))/2 - I3_12*u1^2*cos(2*q2 + 2*q3) - (I3_1*u1^2*sin(2*q2 + 2*q3))/2 + (I3_2*u1^2*sin(2*q2 + 2*q3))/2 + (g*l3*m3*cos(q2 + q3))/2 + (g*l2*m2*cos(q2))/2 + g*l2*m3*cos(q2) + (l2^2*m2*u1^2*sin(2*q2))/8 + (l2^2*m3*u1^2*sin(2*q2))/2 + (l3^2*m3*u1^2*sin(2*q2 + 2*q3))/8 - (l2*l3*m3*u3^2*sin(q3))/2 + (l2*l3*m3*u1^2*sin(2*q2 + q3))/2 - l2*l3*m3*u2*u3*sin(q3); 
RHS3 = fy*l3 - T3 - I3_12*u1^2*cos(2*q2 + 2*q3) - (I3_1*u1^2*sin(2*q2 + 2*q3))/2 + (I3_2*u1^2*sin(2*q2 + 2*q3))/2 + (g*l3*m3*cos(q2 + q3))/2 + (l3^2*m3*u1^2*sin(2*q2 + 2*q3))/8 + (l2*l3*m3*u1^2*sin(q3))/4 + (l2*l3*m3*u2^2*sin(q3))/2 + (l2*l3*m3*u1^2*sin(2*q2 + q3))/4; 

MM  = [M11 M12 M13;                               
       M21 M22 M23;                               
       M31 M32 M33];                              

RHS = [RHS1; RHS2; RHS3];                      

X = MM \ RHS;                                    

ud1 = X(1);                                       
ud2 = X(2);                                     

ud3 = X(3);                                     

zdot = [u1 u2 u3 ud1 ud2 ud3]';
