
function ddq = boom_partial(u)

th1 = u(1);
th2 = u(2);
d6 = u(3);
th3 = u(4);
th4 = u(5);

dth1 = u(6);
dth2 = u(7);
dd6 = u(8);
dth3 = u(9);
dth4 = u(10);

q = [th1 th2 d6 th3 th4]';
dq = [dth1 dth2 dd6 dth3 dth4]';

I_tot=200;   l_B=8.9;     m_B=312.2;     I_B=2068;  g=9.81;       m=50;

%%%%%%%%%%%%%%5


M = [ I_tot + d6^2*m + l_B^2*m*cos(th2)^2 + (l_B^2*m_B*cos(th2)^2)/4 - d6^2*m*cos(th3)^2*cos(th4)^2 + 2*d6*l_B*m*cos(th2)*sin(th4),                         d6*l_B*m*cos(th4)*sin(th2)*sin(th3),                             l_B*m*cos(th2)*cos(th4)*sin(th3), d6*m*cos(th3)*cos(th4)*(l_B*cos(th2) + d6*sin(th4)),                -d6*m*sin(th3)*(d6 + l_B*cos(th2)*sin(th4));
                                                                                          d6*l_B*m*cos(th4)*sin(th2)*sin(th3),                               I_B + l_B^2*m + (l_B^2*m_B)/4, - l_B*m*sin(th2)*sin(th4) - l_B*m*cos(th2)*cos(th3)*cos(th4),                 d6*l_B*m*cos(th2)*cos(th4)*sin(th3), -d6*l_B*m*(cos(th4)*sin(th2) - cos(th2)*cos(th3)*sin(th4));
                                                                                             l_B*m*cos(th2)*cos(th4)*sin(th3), -m*(l_B*sin(th2)*sin(th4) + l_B*cos(th2)*cos(th3)*cos(th4)),                                                            m,                                                   0,                                                          0;
                                                                          d6*m*cos(th3)*cos(th4)*(l_B*cos(th2) + d6*sin(th4)),                         d6*l_B*m*cos(th2)*cos(th4)*sin(th3),                                                            0,                            -d6^2*m*(sin(th4)^2 - 1),                                                          0;
                                                                                  -d6*m*sin(th3)*(d6 + l_B*cos(th2)*sin(th4)),  -d6*l_B*m*(cos(th4)*sin(th2) - cos(th2)*cos(th3)*sin(th4)),                                                            0,                                                   0,                                                     d6^2*m];
                                                                              
                                                                              
                                                                              
 %%%%%%%%%%%%
 
 C = [                                                                              2*d6*dd6*m - dth2*l_B^2*m*sin(2*th2) - (dth2*l_B^2*m_B*sin(2*th2))/4 - 2*d6*dd6*m*cos(th3)^2*cos(th4)^2 + 2*dd6*l_B*m*cos(th2)*sin(th4) + 2*d6*dth4*l_B*m*cos(th2)*cos(th4) - 2*d6*dth2*l_B*m*sin(th2)*sin(th4) + 2*d6^2*dth3*m*cos(th3)*cos(th4)^2*sin(th3) + 2*d6^2*dth4*m*cos(th3)^2*cos(th4)*sin(th4),                                                                                                                                                                      dd6*l_B*m*cos(th4)*sin(th2)*sin(th3) + d6*dth2*l_B*m*cos(th2)*cos(th4)*sin(th3) + d6*dth3*l_B*m*cos(th3)*cos(th4)*sin(th2) - d6*dth4*l_B*m*sin(th2)*sin(th3)*sin(th4),                                                                                                                   dth3*l_B*m*cos(th2)*cos(th3)*cos(th4) - dth2*l_B*m*cos(th4)*sin(th2)*sin(th3) - dth4*l_B*m*cos(th2)*sin(th3)*sin(th4), d6*m*cos(th3)*cos(th4)*(dd6*sin(th4) - dth2*l_B*sin(th2) + d6*dth4*cos(th4)) + dd6*m*cos(th3)*cos(th4)*(l_B*cos(th2) + d6*sin(th4)) - d6*dth3*m*cos(th4)*sin(th3)*(l_B*cos(th2) + d6*sin(th4)) - d6*dth4*m*cos(th3)*sin(th4)*(l_B*cos(th2) + d6*sin(th4)),                                                                                                                                                - dd6*m*sin(th3)*(d6 + l_B*cos(th2)*sin(th4)) - d6*m*sin(th3)*(dd6 + dth4*l_B*cos(th2)*cos(th4) - dth2*l_B*sin(th2)*sin(th4)) - d6*dth3*m*cos(th3)*(d6 + l_B*cos(th2)*sin(th4));
                                                                                  (dth1*(2*l_B^2*m*cos(th2)*sin(th2) + (l_B^2*m_B*cos(th2)*sin(th2))/2 + 2*d6*l_B*m*sin(th2)*sin(th4)))/2 + (3*dd6*l_B*m*cos(th4)*sin(th2)*sin(th3))/2 + (d6*dth2*l_B*m*cos(th2)*cos(th4)*sin(th3))/2 + (3*d6*dth3*l_B*m*cos(th3)*cos(th4)*sin(th2))/2 - (3*d6*dth4*l_B*m*sin(th2)*sin(th3)*sin(th4))/2,                                                                                                      (dd6*m*(l_B*cos(th2)*sin(th4) - l_B*cos(th3)*cos(th4)*sin(th2)))/2 + (d6*dth4*l_B*m*(cos(th2)*cos(th4) + cos(th3)*sin(th2)*sin(th4)))/2 - (d6*dth1*l_B*m*cos(th2)*cos(th4)*sin(th3))/2 + (d6*dth3*l_B*m*cos(th4)*sin(th2)*sin(th3))/2, (dth2*l_B*m*cos(th3)*cos(th4)*sin(th2))/2 - dth4*l_B*m*cos(th4)*sin(th2) - (dth2*l_B*m*cos(th2)*sin(th4))/2 + dth3*l_B*m*cos(th2)*cos(th4)*sin(th3) + dth4*l_B*m*cos(th2)*cos(th3)*sin(th4) + (dth1*l_B*m*cos(th4)*sin(th2)*sin(th3))/2,                                  dd6*l_B*m*cos(th2)*cos(th4)*sin(th3) + d6*dth3*l_B*m*cos(th2)*cos(th3)*cos(th4) + (d6*dth1*l_B*m*cos(th3)*cos(th4)*sin(th2))/2 - (d6*dth2*l_B*m*cos(th4)*sin(th2)*sin(th3))/2 - d6*dth4*l_B*m*cos(th2)*sin(th3)*sin(th4), dd6*l_B*m*cos(th2)*cos(th3)*sin(th4) - dd6*l_B*m*cos(th4)*sin(th2) - (d6*dth2*l_B*m*cos(th2)*cos(th4))/2 + d6*dth4*l_B*m*sin(th2)*sin(th4) + d6*dth4*l_B*m*cos(th2)*cos(th3)*cos(th4) - (d6*dth2*l_B*m*cos(th3)*sin(th2)*sin(th4))/2 - d6*dth3*l_B*m*cos(th2)*sin(th3)*sin(th4) - (d6*dth1*l_B*m*sin(th2)*sin(th3)*sin(th4))/2;
                                                                                                           d6*dth4*m*sin(th3) - d6*dth1*m + d6*dth1*m*cos(th3)^2*cos(th4)^2 - dth1*l_B*m*cos(th2)*sin(th4) + (dth3*l_B*m*cos(th2)*cos(th3)*cos(th4))/2 - d6*dth3*m*cos(th3)*cos(th4)*sin(th4) - (3*dth2*l_B*m*cos(th4)*sin(th2)*sin(th3))/2 - (dth4*l_B*m*cos(th2)*sin(th3)*sin(th4))/2,                                                                                                dth2*l_B*m*cos(th3)*cos(th4)*sin(th2) - (dth4*l_B*m*cos(th4)*sin(th2))/2 - dth2*l_B*m*cos(th2)*sin(th4) + (dth3*l_B*m*cos(th2)*cos(th4)*sin(th3))/2 + (dth4*l_B*m*cos(th2)*cos(th3)*sin(th4))/2 - (dth1*l_B*m*cos(th4)*sin(th2)*sin(th3))/2,                                                                                                                                                                                                                                       0,                                                                                                                      d6*dth3*m*(sin(th4)^2 - 1) - (dth1*m*cos(th3)*cos(th4)*(l_B*cos(th2) + 2*d6*sin(th4)))/2 - (dth2*l_B*m*cos(th2)*cos(th4)*sin(th3))/2,                                                                                                                                                                                               (dth2*l_B*m*(cos(th4)*sin(th2) - cos(th2)*cos(th3)*sin(th4)))/2 - d6*dth4*m + (dth1*m*sin(th3)*(2*d6 + l_B*cos(th2)*sin(th4)))/2;
 2*d6^2*dth4*m*cos(th3)*cos(th4)^2 - (d6^2*dth4*m*cos(th3))/2 + (dd6*l_B*m*cos(th2)*cos(th3)*cos(th4))/2 + 2*d6*dd6*m*cos(th3)*cos(th4)*sin(th4) - (d6^2*dth3*m*cos(th4)*sin(th3)*sin(th4))/2 - d6^2*dth1*m*cos(th3)*cos(th4)^2*sin(th3) - (3*d6*dth2*l_B*m*cos(th3)*cos(th4)*sin(th2))/2 - (d6*dth3*l_B*m*cos(th2)*cos(th4)*sin(th3))/2 - (d6*dth4*l_B*m*cos(th2)*cos(th3)*sin(th4))/2,                                                                                                           (dd6*l_B*m*cos(th2)*cos(th4)*sin(th3))/2 + (d6*dth3*l_B*m*cos(th2)*cos(th3)*cos(th4))/2 - (d6*dth1*l_B*m*cos(th3)*cos(th4)*sin(th2))/2 - d6*dth2*l_B*m*cos(th4)*sin(th2)*sin(th3) - (d6*dth4*l_B*m*cos(th2)*sin(th3)*sin(th4))/2,                                                                                                                                                                            -(l_B*m*cos(th2)*cos(th4)*(dth1*cos(th3) + dth2*sin(th3)))/2,                                                                               (d6*dth1*m*cos(th4)*sin(th3)*(l_B*cos(th2) + d6*sin(th4)))/2 - 2*d6^2*dth4*m*cos(th4)*sin(th4) - 2*d6*dd6*m*(sin(th4)^2 - 1) - (d6*dth2*l_B*m*cos(th2)*cos(th3)*cos(th4))/2,                                                                                                                                                                                                                             (d6*dth1*m*cos(th3)*(d6 + l_B*cos(th2)*sin(th4)))/2 + (d6*dth2*l_B*m*cos(th2)*sin(th3)*sin(th4))/2;
                                (3*d6*dth2*l_B*m*sin(th2)*sin(th3)*sin(th4))/2 - (d6^2*dth3*m*cos(th3))/2 - d6^2*dth3*m*cos(th3)*cos(th4)^2 - (dd6*l_B*m*cos(th2)*sin(th3)*sin(th4))/2 - d6*dth1*l_B*m*cos(th2)*cos(th4) - d6^2*dth1*m*cos(th3)^2*cos(th4)*sin(th4) - (d6*dth3*l_B*m*cos(th2)*cos(th3)*sin(th4))/2 - (d6*dth4*l_B*m*cos(th2)*cos(th4)*sin(th3))/2 - 2*d6*dd6*m*sin(th3), (dd6*l_B*m*cos(th2)*cos(th3)*sin(th4))/2 - (dd6*l_B*m*cos(th4)*sin(th2))/2 - d6*dth2*l_B*m*cos(th2)*cos(th4) + (d6*dth4*l_B*m*sin(th2)*sin(th4))/2 + (d6*dth4*l_B*m*cos(th2)*cos(th3)*cos(th4))/2 - d6*dth2*l_B*m*cos(th3)*sin(th2)*sin(th4) - (d6*dth3*l_B*m*cos(th2)*sin(th3)*sin(th4))/2 + (d6*dth1*l_B*m*sin(th2)*sin(th3)*sin(th4))/2,                                                                                                                       (dth2*(l_B*m*cos(th4)*sin(th2) - l_B*m*cos(th2)*cos(th3)*sin(th4)))/2 + (dth1*l_B*m*cos(th2)*sin(th3)*sin(th4))/2,                                                                                                      d6^2*dth3*m*cos(th4)*sin(th4) + (d6*dth1*m*cos(th3)*(d6 - 2*d6*cos(th4)^2 + l_B*cos(th2)*sin(th4)))/2 + (d6*dth2*l_B*m*cos(th2)*sin(th3)*sin(th4))/2,                                                                                                                                                                                                 2*d6*dd6*m - (d6*dth2*l_B*m*(sin(th2)*sin(th4) + cos(th2)*cos(th3)*cos(th4)))/2 + (d6*dth1*l_B*m*cos(th2)*cos(th4)*sin(th3))/2];
 
 
 
 
 
 %%%%%%%%%%%%
 

 G =                        [ 0;
 (g*l_B*cos(th2)*(2*m + m_B))/2;
         -g*m*cos(th3)*cos(th4);
       d6*g*m*cos(th4)*sin(th3);
       d6*g*m*cos(th3)*sin(th4)];

 %%%%%%%%%%%%%%%%%%%5
 
 
M11 = M(1:3,1:3);
M12 = M(1:3,4:5);
M21 = M(4:5,1:3);
M22 = M(4:5,4:5);
C11 = C(1:3,1:3);
C12 = C(1:3,4:5);
C21 = C(4:5,1:3);
C22 = C(4:5,4:5);
G1 = G(1:3);
G2 = G(4:5);

M_bar = M11-M12*inv(M22)*M21;
C_bar1 = C11 - M12*inv(M22)*C21;
C_bar2 = C12 - M12*inv(M22)*C22;
G_bar = G1 - M12*inv(M22)*G2;

Kad = diag([100 100 100]);
Kap = diag([10 10 10]);

Kud = diag([100 100]);
Kup = diag([120 120]);

a1 = -1;
a2 = sign(q(2));
alp = [a1 0;0 a2;0 0];

err = [q(1)-u(11);q(2)-u(12);q(3)-u(13)]; 

Va = - Kad*dq(1:3) - Kap*err;
Vu = - Kud*dq(4:5) - Kup*q(4:5);


V = Va+alp*Vu;
U_bar = M_bar*V+C_bar1*dq(1:3)+C_bar2*dq(4:5)+G_bar;
% U_bar(1) = U_bar(1)+u(15) 
% U_bar(2) = U_bar(2)+u(14) 
U_bar(4) = 0;
U_bar(5) = 0;

if U_bar(2) >= 18200
    U_bar(2) = 18200;
else if U_bar(2) <= -18200
        U_bar(2) = -18200;
    end
end

% if U_bar(1) >= 600
%     U_bar(1) = 600;
% else if U_bar(1) <= -600
%         U_bar(1) = -600;
%     end
% end

% if U_bar(3) >= 550
%     U_bar(3) = 550;
% else if U_bar(3) <= -550
%         U_bar(3) = -550;
%     end
% end


% U_bar = [u(15) u(14) 0 -G(4) -G(5)]';

ddq(1:5) = inv(M)*(-C*dq-G+U_bar);
ddq(6:8) = U_bar(1:3)';
