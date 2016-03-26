function [q1,q2,q3,q4,q5,q6] = move_Scara(O1,O2, O3,I1,J1,K1)
%#eml

T = 1;

CalcPoints = 0:0.02:T;

steps = size(CalcPoints,2);




EndPos = [I1;J1;K1];  %[1;0;0];
o6 = [O1;O2;O3];%[1;1;0];

path = EndPos - o6;

d = norm(path);

% Calculate Velocity
v = sin(CalcPoints.*(pi/T));
v = v .*pi* d/(2*T);

v_vector = path/d;

v_vectors = v_vector * v;

qStart = [0; -pi/2; 0; 0; 0; 0];

q = zeros(size(CalcPoints,2), 6);
qDot = q;
qDotDot = q;

q(1,:) = qStart;
seconds = 0.02;

qDot(1,:) = pinv(ScaraJacobian(q(1,:)'))*[v_vectors(:,1);zeros(3,1)];
q(2,:) = qDot(1,:)*0.02 + q(1,:);
    q1 = q(1,1);
    q2 = q(1,2);
    q3 = q(1,3);
    q4 = q(1,4);
    q5 = q(1,5);
    q6 = q(1,6);
for i = 1:steps
    %Calculate velocity
    qDot(i,:) = pinv(ScaraJacobian(q(i,:)'))*[v_vectors(:,i);zeros(3,1)];
    q1 = q(i+1,1);
    q2 = q(i+1,2);
    q3 = q(i+1,3);
    q4 = q(i+1,4);
    q5 = q(i+1,5);
    q6 = q(i+1,6);
    %Calculate new position
    q(i+1,:) = qDot(i,:)*0.02 + q(i,:); 
    
end


