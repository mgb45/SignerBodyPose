function r = rotEuler(a,b,c)

r1 = [1 0 0; 
	0 cos(a) -sin(a); 
	0 sin(a) cos(a)];
	
	
r2	= [cos(b) 0 sin(b);
		0 1 0; 
		-sin(b) 0 cos(b)];
		
r3	= [cos(c) -sin(c) 0;
		sin(c) cos(c) 0;
		0 0 1];
		
r = r3*r2*r1;

