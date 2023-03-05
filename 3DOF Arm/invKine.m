function q=invKine(p,l)

x = p(1,:); y = p(2,:); z = p(3,:); 
l1 = l(1);  l2 = l(2);  l3 = l(3);


q= [];

for i = 1:size(x,2)	 %First Solution 
	%First Solution
     
    r = sqrt(x(i).^2+y(i).^2);
    q1 = atan2(y(i),x(i));

    q2 = 2.*atan((2.*l2.*z(i) + (2.*l2.^2.*r.^2 + 2.*l3.^2.*r.^2 + 2.*l2.^2.*z(i).^2 + 2.*l3.^2.*z(i).^2 - 2.*r.^2.*z(i).^2 - l2.^4 - l3.^4 - r.^4 - z(i).^4 + 2.*l2.^2.*l3.^2).^(1/2))./(2.*l2.*r + l2.^2 - l3.^2 + r.^2 + z(i).^2)); 
	q3 = -2.*atan(((2.*l2.*l3 - l2.^2 - l3.^2 + r.^2 + z(i).^2).*(2.*l2.*l3 + l2.^2 + l3.^2 - r.^2 - z(i).^2)).^(1/2)./(2.*l2.*l3 - l2.^2 - l3.^2 + r.^2 + z(i).^2)); 
    
	q = [q; q1 q2 q3]; 

end

end