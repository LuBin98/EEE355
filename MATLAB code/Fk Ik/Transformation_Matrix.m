%forward kinematic equation / transformation matrices
syms Q1 Q2 Q3 Q4 Q5 Q6 D1 D4 A1 A2

Q = [Q1,Q2,Q3,Q4,Q5,Q6];%Joint angle
d = [D1,0,0,D4,0,0];%Joint offset
a = [A1,A2,0,0,0,0];%Link length
alphaa = [90,0,90,90,-90,0];%Twist angle

for i=1:6
   switch i
       case 1
           T01= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
       case 2
           T12= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
       case 3
           T23= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
       case 4
           T34= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
       case 5
           T45= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
       case 6
           T5H= [cos(Q(1,i)),-sin(Q(1,i))*cosd(alphaa(1,i)),sind(alphaa(1,i))*sin(Q(1,i)),a(1,i)*cos(Q(1,i));sin(Q(1,i)),cos(Q(1,i)).*cosd(alphaa(1,i)),-sind(alphaa(1,i))*cos(Q(1,i)),sin(Q(1,i))*a(1,i);0,sind(alphaa(1,i)),cosd(alphaa(1,i)),d(1,i);0,0,0,1];
   end
end

T01
T12
T23
T34
T45
T5H
T0H = simplify(T01*T12*T23*T34*T45*T5H)

nx = T0H(1,1)
ny = T0H(2,1)
nz = T0H(3,1)

ox = T0H(1,2)
oy = T0H(2,2)
oz = T0H(3,2)

ax = T0H(1,3)
ay = T0H(2,3)
az = T0H(3,3)

%jacobian
px = T0H(1,4)
py = T0H(2,4)
pz = T0H(3,4)

dpx1 = simplify(diff(px,Q1))
dpx2 = simplify(diff(px,Q2))
dpx3 = simplify(diff(px,Q3))
dpx4 = simplify(diff(px,Q4))
dpx5 = simplify(diff(px,Q5))
dpx6 = simplify(diff(px,Q6))

dpy1 = simplify(diff(py,Q1))
dpy2 = simplify(diff(py,Q2))
dpy3 = simplify(diff(py,Q3))
dpy4 = simplify(diff(py,Q4))
dpy5 = simplify(diff(py,Q5))
dpy6 = simplify(diff(py,Q6))

dpz1 = simplify(diff(pz,Q1))
dpz2 = simplify(diff(pz,Q2))
dpz3 = simplify(diff(pz,Q3))
dpz4 = simplify(diff(pz,Q4))
dpz5 = simplify(diff(pz,Q5))
dpz6 = simplify(diff(pz,Q6))

J = [dpx1 dpx2 dpx3 dpx4 dpx5 dpx6;
     dpy1 dpy2 dpy3 dpy4 dpy5 dpy6;
     dpz1 dpz2 dpz3 dpz4 dpz5 dpz6]
 
 syms a1 a2 a3 b1 b2 b3 c1 c2 c3 d1 d2 d3
W = [ a1 b1 c1 d1;
      a2 b2 c2 d2;
      a3 b3 c3 d3;
       0  0  0  1]
X1 = simplify(inv(T01)*W)
Z1 = simplify(T12*T23*T34*T45*T5H)

X2 = simplify(inv(T23)*inv(T12)*inv(T01)*W)
Z2 = simplify(T34*T45*T5H)

X3 = simplify(inv(T34)*inv(T23)*inv(T12)*inv(T01)*W)
Z3 = simplify(T45*T5H)