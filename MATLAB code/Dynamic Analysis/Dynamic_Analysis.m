%%Modified from online resources
%%Denavit-Hatenberg Parameters
% i  thi  di    ai    alphai
% 1  th1  400   180    pi/2  
% 2  th2  0     600     0 
% 3  th3  0     0      pi/2
% 4  th4  600   0      pi/2
% 5  th5  0     0      -pi/2
% H  th6  0     0        0
syms th1 th2 th3 th4 th5 th6 l0 l1 l2 l3 l4;
% distance from one joint to the other, units in metre
l1=0.52; l4=0.8; a1=0.16; a2=0.78; 
% Transformation matrix
% base at midpoint between 2 arms
AR= [1 0 0 0; 0 1 0 0; 0 0 1 0; 0 0 0 1];
A1= [cos(th1) 0 sin(th1) a1*cos(th1); sin(th1) 0 -cos(th1) a1*cos(th1); 0 1 0 l1; 0 0 0 1];
A2= [cos(th2) -sin(th2) 0 a2*cos(th2); sin(th2) cos(th2) 0 a2*sin(th2); 0 0 1 0; 0 0 0 1];
A3= [cos(th3) 0 sin(th3) 0; sin(th3) 0 -cos(th3) 0; 0 1 0 0; 0 0 0 1];
A4= [cos(th4) 0 sin(th4) 0; sin(th4) 0 -cos(th4) 0; 0 1 0 l4; 0 0 0 1];
A5= [cos(th5) 0 -sin(th5) 0; sin(th5) 0 cos(th5) 0; 0 -1 0 0; 0 0 0 1];
A6= [cos(th6) -sin(th6) 0 0; sin(th6) cos(th6) 0 0; 0 0 1 0; 0 0 0 1];
% Transformation from base to each link
AR1=A1;
AR2=AR1*A2;
AR3=AR2*A3;
AR4=AR3*A4;
AR5=AR4*A5;
AR6=AR5*A6;
% Links Specifications, obtained from solidworks
% Mass in kg [assume density is 1000kg/m^3)
m1=0.0201; m2=13.53; m3=12.1; m4=2.44; m5=1.74; m6=0.314;
% position of centre of mass [x,y,z,0]
r1=[0.147; 0.00066; 0.163; 0];
r2=[0.00015; 0.308; 0.0547; 0];
r3=[-0.367; 0.00966; 0.0177; 0];
r4=[0.00002; 0.00635; -0.109; 0];
r5=[0.0521; -0.00019; -0.0478; 0];
r6=[0.285; -1.18; 1.43; 0];

   %Uij Matrixes
   U11=diff(AR1,th1);   U21=diff(AR2,th1);  U31=diff(AR3,th1);
   U12=zeros(4);        U22=diff(AR2,th2);  U32=diff(AR3,th2);
   U13=zeros(4);        U23=zeros(4);       U33=diff(AR3,th3);
   U14=zeros(4);        U24=zeros(4);       U34=zeros(4);         
   U15=zeros(4);        U25=zeros(4);       U35=zeros(4);                 
   U16=zeros(4);        U26=zeros(4);       U36=zeros(4);
   
   U41=diff(AR4,th1);   U51=diff(AR5,th1);  U61=diff(AR6,th1);
   U42=diff(AR4,th2);   U52=diff(AR5,th2);  U62=diff(AR6,th2);
   U43=diff(AR4,th3);   U53=diff(AR5,th3);  U63=diff(AR6,th3);
   U44=diff(AR4,th4);   U54=diff(AR5,th4);  U64=diff(AR6,th4);
   U45=zeros(4);        U55=diff(AR5,th5);  U65=diff(AR6,th5);
   U46=zeros(4);        U56=zeros(4);       U66=diff(AR6,th6);
  
   %Uijk Matrixes
   
   %U1jk Matrixes
   U111=diff(U11,th1); U121=zeros(4)     ; U131=zeros(4);
   U112=zeros(4)     ; U122=zeros(4)     ; U132=zeros(4);
   U113=zeros(4)     ; U123=zeros(4)     ; U133=zeros(4);
   U114=zeros(4)     ; U124=zeros(4)     ; U134=zeros(4);
   U115=zeros(4)     ; U125=zeros(4)     ; U135=zeros(4);
   U116=zeros(4)     ; U126=zeros(4)     ; U136=zeros(4);
   
   U141=zeros(4)     ; U151=zeros(4)     ; U161=zeros(4);
   U142=zeros(4)     ; U152=zeros(4)     ; U162=zeros(4);
   U143=zeros(4)     ; U153=zeros(4)     ; U163=zeros(4); 
   U144=zeros(4)     ; U154=zeros(4)     ; U164=zeros(4); 
   U145=zeros(4)     ; U155=zeros(4)     ; U165=zeros(4); 
   U146=zeros(4)     ; U156=zeros(4)     ; U166=zeros(4); 
   
   %U2jk Matrixes
   U211=diff(U21,th1); U221=diff(U22,th1); U231=zeros(4);
   U212=diff(U21,th2); U222=diff(U22,th2); U232=zeros(4);
   U213=zeros(4)     ; U223=zeros(4)     ; U233=zeros(4);
   U214=zeros(4)     ; U224=zeros(4)     ; U234=zeros(4);
   U215=zeros(4)     ; U225=zeros(4)     ; U235=zeros(4);
   U216=zeros(4)     ; U226=zeros(4)     ; U236=zeros(4);
   
   U241=zeros(4)     ; U251=zeros(4)     ; U261=zeros(4); 
   U242=zeros(4)     ; U252=zeros(4)     ; U262=zeros(4); 
   U243=zeros(4)     ; U253=zeros(4)     ; U263=zeros(4); 
   U244=zeros(4)     ; U254=zeros(4)     ; U264=zeros(4); 
   U245=zeros(4)     ; U255=zeros(4)     ; U265=zeros(4); 
   U246=zeros(4)     ; U256=zeros(4)     ; U266=zeros(4);
   
   %U3jk Matrixes
   U311=diff(U31,th1); U321=diff(U32,th1); U331=diff(U33,th1);
   U312=diff(U31,th2); U322=diff(U32,th2); U332=diff(U33,th2);
   U313=diff(U31,th3); U323=diff(U32,th3); U333=diff(U33,th3);
   U314=zeros(4)     ; U324=zeros(4)     ; U334=zeros(4);
   U315=zeros(4)     ; U325=zeros(4)     ; U335=zeros(4);
   U316=zeros(4)     ; U326=zeros(4)     ; U336=zeros(4); 
   
   U341=zeros(4)     ; U351=zeros(4)     ; U361=zeros(4);
   U342=zeros(4)     ; U352=zeros(4)     ; U362=zeros(4); 
   U343=zeros(4)     ; U353=zeros(4)     ; U363=zeros(4); 
   U344=zeros(4)     ; U354=zeros(4)     ; U364=zeros(4); 
   U345=zeros(4)     ; U355=zeros(4)     ; U365=zeros(4); 
   U346=zeros(4)     ; U356=zeros(4)     ; U366=zeros(4);
   
   %U4jk Matrixes
   U411=diff(U41,th1); U421=diff(U42,th1); U431=diff(U43,th1);
   U412=diff(U41,th2); U422=diff(U42,th2); U432=diff(U43,th2);
   U413=diff(U41,th3); U423=diff(U42,th3); U433=diff(U43,th3);
   U414=diff(U41,th4); U424=diff(U42,th4); U434=diff(U43,th4);
   U415=zeros(4);      U425=zeros(4);      U435=zeros(4);
   U416=zeros(4);      U426=zeros(4);      U436=zeros(4);
   
   U441=diff(U44,th1); U451=zeros(4)     ; U461=zeros(4); 
   U442=diff(U44,th2); U452=zeros(4)     ; U462=zeros(4); 
   U443=diff(U44,th3); U453=zeros(4)     ; U463=zeros(4); 
   U444=diff(U44,th4); U454=zeros(4)     ; U464=zeros(4); 
   U445=zeros(4);      U455=zeros(4)     ; U465=zeros(4); 
   U446=zeros(4);      U456=zeros(4)     ; U466=zeros(4);     
   
   %U5jk Matrixes
   U511=diff(U51,th1); U521=diff(U52,th1); U531=diff(U53,th1); 
   U512=diff(U51,th2); U522=diff(U52,th2); U532=diff(U53,th2); 
   U513=diff(U51,th3); U523=diff(U52,th3); U533=diff(U53,th3); 
   U514=diff(U51,th4); U524=diff(U52,th4); U534=diff(U53,th4); 
   U515=diff(U51,th5); U525=diff(U52,th5); U535=diff(U53,th5);      
   U515=diff(U51,th5); U525=diff(U52,th5); U535=diff(U53,th5);
   U516=zeros(4)     ; U526=zeros(4)     ; U536=zeros(4);
   
   U541=diff(U54,th1); U551=diff(U55,th1); U561=zeros(4); 
   U542=diff(U54,th2); U552=diff(U55,th2); U562=zeros(4); 
   U543=diff(U54,th3); U553=diff(U55,th3); U563=zeros(4); 
   U544=diff(U54,th4); U554=diff(U55,th4); U564=zeros(4);     
   U545=diff(U54,th5); U555=diff(U55,th5); U565=zeros(4);         
   U546=zeros(4)     ; U556=zeros(4)     ; U566=zeros(4);  
   
   %U6jk Matrixes
   U611=diff(U61,th1); U621=diff(U62,th1); U631=diff(U63,th1); 
   U612=diff(U61,th2); U622=diff(U62,th2); U632=diff(U63,th2); 
   U613=diff(U61,th3); U623=diff(U62,th3); U633=diff(U63,th3); 
   U614=diff(U61,th4); U624=diff(U62,th4); U634=diff(U63,th4); 
   U615=diff(U61,th5); U625=diff(U62,th5); U635=diff(U63,th5);      
   U616=diff(U61,th6); U626=diff(U62,th6); U636=diff(U63,th6);
 
   U641=diff(U64,th1); U651=diff(U65,th1); U661=diff(U66,th1); 
   U642=diff(U64,th2); U652=diff(U65,th2); U662=diff(U66,th2); 
   U643=diff(U64,th3); U653=diff(U65,th3); U663=diff(U66,th3); 
   U644=diff(U64,th4); U654=diff(U65,th4); U664=diff(U66,th4);     
   U645=diff(U64,th5); U655=diff(U65,th5); U665=diff(U66,th5);         
   U646=diff(U64,th6); U656=diff(U65,th6); U666=diff(U66,th6); 
   
%Moment of Inertia matrixes obtain from Solidworks
  I1=[0.000963 -0.00000697 0.000708;
      -0.00000697 0.00169 -0.00000259;
      0.000708 -0.00000259 0.00098];
  
  I2=[1.87 0.00113 -0.0000531;
      0.00113 0.0966 0.253;
      -0.0000531 0.253 1.83];

  I3=[0.0612 -0.524 -0.1;
     -0.0524 2.06 0.00533;
     -0.1 0.00533 2.07];
 
  I4=[0.0435 0.00000269 -0.00000234;
      0.00000269 0.0432 -0.00294;
      -0.00000234 -0.00294 0.00447];

  I5=[0.0123 -0.0000228 -0.00403;
      -0.0000228 0.0212 0.000037;
      -0.00403 0.000037 0.0114];
  
  I6=[1.08 -0.105 0.128;
      -0.105 0.668 -0.529;
      0.128 -0.529 0.462];
  
  %Construction of J matrix (Pseudo inertia matrix)
  
  I=zeros(4);
  for i=1:6
      I=eval(['I' num2str(i)]);
      m=eval(['m' num2str(i)]);
      r=eval(['r' num2str(i)]);
      eval(['j11' '=((-I(1,1)+I(2,2)+I(3,3))/2)']);
      eval(['j12' '=I(1,2)']);
      eval(['j13' '=I(1,3)']);
      eval(['j14' '=m*r(1)']);
      
      eval(['j21' '=I(1,2)']);
      eval(['j22' '=((I(1,1)-I(2,2)+I(3,3))/2)']);
      eval(['j23' '=I(2,3)']);
      eval(['j24' '=m*r(2)']);
      
      eval(['j31' '=I(1,3)']);
      eval(['j32' '=I(2,3)']);
      eval(['j33' '=((I(1,1)+I(2,2)-I(3,3))/2)']);
      eval(['j34' '=m*r(3)']);
      
      eval(['j41' '=m*r(1)']);
      eval(['j42' '=m*r(2)']);
      eval(['j43' '=m*r(3)']);
      eval(['j44' '=m']);
      
      J=[j11 j12 j13 j14;j21 j22 j23 j24; j31 j32 j33 j34;j41 j42 j43 j44];
      eval(['J' num2str(i) '=J']);
  end
  
    %Construction of D matrix or Dij
  
  Uaux=zeros(4);
for i=1:6
   for j=1:6
       m=max([i j]);
       x=0;
       for k=m:6
       Uaux=eval(['U' num2str(k) num2str(i)]);
       Ud=Uaux';
       A=eval(['U' num2str(k) num2str(j)])*eval(['J' num2str(k)])*Ud;
       x=x+trace(A);
       end
       eval(['d' num2str(i) num2str(j) '=x']);
   end
end

D=[d11 d12 d13 d14 d15 d16;
   d21 d22 d23 d24 d25 d26;
   d31 d32 d33 d34 d35 d36;
   d41 d33 d43 d44 d45 d46;
   d51 d52 d53 d54 d55 d56;
   d61 d62 d63 d64 d65 d66];

%Hikm matrix
  for i=1:6
   for k=1:6
     for m=1:6
       j=max([i m k]);
       x=0;
       for l=j:6
       Uaux=eval(['U' num2str(j) num2str(i)]);
       Uh=Uaux';
       x=x+trace(eval(['U' num2str(j) num2str(k) num2str(m)])*eval(['J' num2str(j)])*Uh);
       end
       eval(['h' num2str(i) num2str(k) num2str(m) '=x']);
     end 
   end
  end
  
   %Coriolis column matrix
 syms dth1 dth2 dth3 dth4 dth5 dth6
 
  for i=1:6
      y=0;
      for k=1:6
        y=y+x;
        x=0;  
          for m=1:6
          y=x+eval(['h' num2str(i) num2str(k) num2str(m)])*eval(['dth' num2str(k)])*eval(['dth' num2str(m)]);
          end    
      end
      eval(['h' num2str(i) '=y']);
  end
  
  H=[h1;h2;h3;h4;h5;h6]; 
  
    %Gravity column matrix
  
  g=[0 0 -9.81 0];
  
  for i=1:6
      x=0;
      for j=i:6
      x=x+(-eval(['m' num2str(j)])*g*eval(['U' num2str(j) num2str(i)])*eval(['r' num2str(j)]));  
      end
      eval(['c' num2str(i) '=x']);
  end
   
  C=[c1;c2;c3;c4;c5;c6];
  
  %Final dynamic model
  
  %angular acceleration matrix
  
  syms ddth1 ddth2 ddth3 ddth4 ddth5 ddth6
  ddth=[ddth1;ddth2;ddth3;ddth4;ddth5;ddth6]; 
  %T=[t1;t2;t3;t4;t5;t6;t7]; %Matrix with torques of each joint
  
  T=(D*ddth+H+C);


