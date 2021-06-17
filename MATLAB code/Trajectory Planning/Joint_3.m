%finding the inverse of M
MINV = inv(M);
C = MINV*A;

%separate the terms get in C into a,b,c,d,e,f,g manually
%sorting the matrix
a = flip(C(1:5,1));
b = flip(C(6:9,1));
c = flip(C(10:14,1));

%define the x and y axes
t = [0 1 2 3 4 5];
y = [-25.58 -26.069 -42.903 -42.771];

%define the local time for each segment
%since having a total of 7 segments (2 arbitrary points)
t1 = [0:0.1:1.0];           %segment 1
t2 = [1.0:0.1:4.0];         %segment 2
t3 = [4.0:0.1:5.0];         %segment 3


%getting the position equation of each segment using the local time
y1 = polyval(a,(0:0.1:1.0));    %segment 1
y2 = polyval(b,(0:0.1:3.0));    %segment 2
y3 = polyval(c,(0:0.1:1.0));    %segment 3


%define the velocity equation of each segment
a_1 = polyder(a);              
b_1 = polyder(b);
c_1 = polyder(c);

%calculate the velocity of each segment
y1_v = polyval(a_1,[0:0.1:1.0]);    %segment 1
y2_v = polyval(b_1,[0:0.1:3.0]);    %segment 2
y3_v = polyval(c_1,[0:0.1:1.0]);    %segment 3

%define the acceleration equation of each segment
a_2 = polyder(a_1);
b_2 = polyder(b_1);
c_2 = polyder(c_1);

%calculate the acceleration of each segment
y1_a = polyval(a_2,[0:0.1:1.0]);    %segment 1
y2_a = polyval(b_2,[0:0.1:3.0]);    %segment 2
y3_a = polyval(c_2,[0:0.1:1.0]);    %segment 3

%plotting
%Position
figure(1)
hold on;
plot(t1,y1,'r');
plot(t2,y2,'g');
plot(t3,y3,'b');

title('Joint Angle Profile');
xlabel('Time(s)');
ylabel('θ(°)');
hold off;

%Velocity
figure(2);
hold on;
plot(t1,y1_v,'r');
plot(t2,y2_v,'g');
plot(t3,y3_v,'b');

title('Velocity Profile');
xlabel('Time(s)');
ylabel('V(m/s)');
hold off;

%Acceleration
figure(3);
hold on;
plot(t1,y1_a,'r');
plot(t2,y2_a,'g');
plot(t3,y3_a,'b');

title('Acceleration Profile');
xlabel('Time(s)');
ylabel('A(m/s^2)');
hold off;