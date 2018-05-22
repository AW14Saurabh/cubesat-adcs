% Reaction Wheel Module v3 Simulator
% Mark Yeo; mark.yeo@student.unsw.edu.au
% Last modified 2018/04/14

function RWSModel()
    clear;
    Pr = initParams();
    St = initStateModel(Pr);
    Datalog = initDatalog(Pr);
    
    for iteration = 1:Pr.simIters
        St = detumbleControllerQuat(St,Pr);
        %St = pointController(St,Pr);
        St = calcWheelSpeeds(St,Pr);
        St = updateStateModel(St,Pr);
        Datalog = updateDatalog(St,Datalog,iteration);
    end
    plotDatalog(Pr,Datalog);
end

function Pr = initParams() % things that don't change
    Pr.satThStart = angle2quat(0,0,0);	% initial heading angle of RWS (quaternion)
    Pr.satWStart = [1.0,2.0,3.0];             % initial angular velocity of RWS
    Pr.motorWStart = [0,0,0,0];        % initial motor speeds (1x scalar per motor)
    
    Pr.wheelI = 1.41E-05;               % mass moment of inertia of motor + wheel (motor est. as uniform cylinder)
    Pr.satI = 2.2E-03;                	% mass moment of inertia of 1U satellite (approximated as a uniform 1.33kg 10cm^3 cube)
    Pr.simDt = 0.001;                     % simulation time step (s)
    Pr.simDuration = 0.5;                 % length of simulation (s)
    Pr.motorMaxW = 13300/60*360/180*pi;  % 2610T006B SC motor max speed (rad/s) %update this from characterisation data
    Pr.targetTh = 90/180*pi;
    Pr.motorMaxT = 6*0.001;         % max torque of motor (N)
    %implied parameters
    Pr.simIters = Pr.simDuration / Pr.simDt;    % number of iterations (calculated)
end

function St = initStateModel(Pr) %things that change each cycle
    St.satTh = Pr.satThStart;
    St.satW = Pr.satWStart;
    St.motorW = Pr.motorWStart; %angular velocity of each motor (4x scalars)
    St.reqWheelCombDH = [0,0,0];   %required change in H each iteration
    St.reqWheelCombH = [0,0,0];    %required combined angular momentum in x,y,z
    St.wheelCombDH = [0,0,0]; %actual wheel combined dH (from dW of motors)
    St.errorPrev = 0;
    St.errorCumul = 0;
end

function Datalog = initDatalog(Pr)
    Datalog.satTh = zeros(Pr.simIters,4);
    Datalog.satW = zeros(Pr.simIters,3);
    Datalog.motorW = zeros(Pr.simIters,4);
end

function Datalog = updateDatalog(St,Datalog,iteration)
    Datalog.satTh(iteration,:) = St.satTh;
    Datalog.satW(iteration,:) = St.satW;
    Datalog.motorW(iteration,:) = St.motorW;
end

function St = detumbleControllerQuat(St,Pr)
    %calculate the ideal change in angular momentum required
    
    % simple P controller
    error = 0-St.satW;
    P = -0.05;
    St.reqWheelCombDH = P*error;
    
    %normalise for dT
    St.reqWheelCombDH = St.reqWheelCombDH * Pr.simDt;
    
    %detumble idea: set rws angular momentum (H) = - sat H
    %ideally would this be done in quat space??
end

function St = pointController(St,Pr)
    % points around a single predetermined axis
    % PID controller
    theta = quat2eul(St.satTh);
    axis = 1; %yaw pitch roll
    error = Pr.targetTh-theta(axis);
    P = -0.1;
    I = -0.0000001;
    D = -0.04;
    %calculate D & I terms
    dError = (error - St.errorPrev)/Pr.simDt;
    St.errorPrev = error;
    St.errorCumul = St.errorCumul + error*Pr.simDt;
    axisDH = P*error + I*St.errorCumul + D*dError;
    
    %normalise for sim dT
    axisDH = axisDH*Pr.simDt;
    St.reqWheelCombDH = [0,0,0];
    St.reqWheelCombDH(4-axis) = axisDH;
end



function St = pointControllerQuat(St,Pr)
    %put on hold until later if u have time
    %{
    find axis to accelerate around
    accelerate until halfway
    decelerate in same direction
    detumble when angular velocity below threshold
    repeat
    %}
    
    
    % errorquat = quatmultiply(targetquat, inverse(currentquat))
    quatnormalize(St.satTh);
    errorQuat = quatmultiply([1 0 0 0],quatconj(St.satTh));
    errorDir = errorQuat(2:4);
    
    %{
    reqWheelCombDH in direction of error, magnitude theta (2arccos(errorQuat))
    
    3x PIDs: 1 azimuth 1 elevation 1 angle (axis-angle)?
    3x PIDs for each axis?
    Some multi-input-multi-output controller?
    
    %}

    %normalise for sim dT
    St.reqWheelCombDH = St.reqWheelCombDH*Pr.simDt;

end







function St = calcWheelSpeeds(St,Pr)
    %calculate the wheel speeds (and combined dH) given the required
    % combined dH and the physical motor limits
    
    %calculate pseudo-inverse to get ideal wheel speeds
    A = [
        0           sqrt(2)     -1; %1
        -sqrt(2)    0           -1; %2
        0           -sqrt(2)    -1  %3
        sqrt(2)    0           -1; %4
        ];
    ZPI = sqrt(3)/4*A;
    motorDH = (ZPI * St.reqWheelCombDH')';
    
    
    % motor can only change H as fast as torque*dt, so scale down motorDH
    %  if any motor exceeds its torque limit
    torqueMax = Pr.motorMaxT*Pr.simDt;
    if (max(motorDH) > torqueMax)
        motorDH = motorDH/max(motorDH)*torqueMax;
    end
    
    % convert to angular velocity
    motorDW = motorDH/Pr.wheelI;

    % if motor is saturated, no change in motorW
    if (max(abs(St.motorW + motorDW)) > Pr.motorMaxW)
        motorDW = [0,0,0,0];
    end
    
    St.motorW = St.motorW + motorDW;
    
    motorDH = motorDW * Pr.wheelI;
    St.wheelCombDH = (1/sqrt(3)*A'*motorDH')';
    %a = St.reqWheelCombDH
    %b = St.wheelCombDH

end





function St = updateStateModel(St,Pr)
    % calculate w
    St.satW = St.satW - St.wheelCombDH./Pr.satI;
    
    % calculate dTheta (quat version of dTheta = w*dt)
    dThNr = quatnormalize([1,St.satW * Pr.simDt *0.5]);
    
    % add dTheta to Theta
    St.satTh = quatmultiply(St.satTh, dThNr);
    
    % add motordW to reqWheelCombH
    St.reqWheelCombH = St.reqWheelCombH + St.reqWheelCombDH/Pr.wheelI;
end

function plotDatalog(Pr,Datalog)
    figure(1);
    % graph sat w (roll pitch yaw)
    subplot(2,3,1);
    t = linspace(0,Pr.simDuration, Pr.simIters);
    satW = Datalog.satW*180/pi;
    hold off;
    plot(t,satW(:,1),'color', [0.8 0.8 0]); %yaw
    hold on;
    plot(t,satW(:,2),'m');                  %pitch
    plot(t,satW(:,3),'r');                  %roll
    title('Sat w');
    %axis([0,Pr.simDuration,-180,180]);
    hold off;
    
    
    % graph sat heading angle (roll pitch yaw)
    subplot(2,3,2);
    t = linspace(0,Pr.simDuration, Pr.simIters);
    satTh = quat2eul(Datalog.satTh)*180/pi;
    hold off;
    plot(t,satTh(:,1),'color', [0.8 0.8 0]); %yaw
    hold on;
    plot(t,satTh(:,2),'m');                 %pitch
    plot(t,satTh(:,3),'r');                 %roll
    title('Sat Th');
    axis([0,Pr.simDuration,-180,180]);
    
    % graph error in sat heading angle (roll pitch yaw)
    subplot(2,3,5);
    t = linspace(0,Pr.simDuration, Pr.simIters);
    satTh = quat2eul(Datalog.satTh)*180/pi;
    hold off;
    plot(t,90-satTh(:,1),'color', [0.8 0.8 0]); %yaw
    hold on;
    plot(t,satTh(:,2),'m');                 %pitch
    plot(t,satTh(:,3),'r');                 %roll
    title('Sat Th Error');
    %axis([0,Pr.simDuration,-180,180]);
    
    
    % graph heading angle over time
    subplot(2,3,4);
    %+ve yaw is anticlockwise??
    nth = 50;   %inverse to density of arrows
    vrot = quatrotate(Datalog.satTh, [1,0,0]);
    arrowX = vrot(:,1);
    arrowY = vrot(:,2);
    arrowZ = vrot(:,3);
    Os = zeros(Pr.simIters,1);
    quiver3(Os(nth:nth:end),Os(nth:nth:end),Os(nth:nth:end),arrowX(nth:nth:end),arrowY(nth:nth:end),arrowZ(nth:nth:end),1);
    title('RW-ADCS Heading');
    axis([-1,1,-1,1,-1,1]);
    hold on;
    
    
    % graph wheel speeds over time
    subplot(2,3,3);
    t = linspace(0,Pr.simDuration, Pr.simIters);
    hold off;
    plot(t,Datalog.motorW(:,1),'r'); %motor 1
    hold on;
    plot(t,Datalog.motorW(:,2),'g'); %motor 2
    plot(t,Datalog.motorW(:,3),'b'); %motor 3
    plot(t,Datalog.motorW(:,4),'color', [0.8 0.8 0]); %motor 3
    title('Motor Ws');
    %axis([0,Pr.simDuration,-180,180]);
    
    %Animate RWS heading:
    %{
    for i = 1:Pr.simIters
        % trace line through vector tips:
        plot3(arrowX(1:i),arrowY(1:i),arrowZ(1:i));
        
        
        % arrow for each timestep:
        %quiver3(zeros(i,1),zeros(i,1),zeros(i,1),arrowX(1:i),arrowY(1:i),arrowZ(1:i),1);
        
        
        % single moving arrow:
        %quiver3(0,0,0,arrowX(i),arrowY(i),arrowZ(i),0.9);
        %hold off;
        %axis([-1,1,-1,1,-1,1]);
        
        pause(Pr.simDt*0.5)
    end
    %}
end






































%Simulink?
%https://au.mathworks.com/help/ident/examples/building-and-estimating-process-models-using-system-identification-toolbox.html
%https://au.mathworks.com/help/simulink/ug/modeling-dynamic-systems.html
%https://au.mathworks.com/help/ident/ug/what-is-a-process-model.html




function St = detumbleController(St,Pr)
    % simple P controller
    error = 0-St.satW(3);
    P = -0.001; %with Pr.simDt = 0.001
    St.reqWheelCombDH = P*error;
    
    % motor can only change H as fast as torque*dt
    torqueMax = 6*0.001*Pr.simDt;
    St.reqWheelCombDH = min(St.reqWheelCombDH, torqueMax);
    St.reqWheelCombDH = max(St.reqWheelCombDH, -torqueMax);
        % maths:
        % T = I*alpha
        % dw = alpha*dt
        % H = I*w
        % dH = I*dw
        % dH = T*dt
    
    % if reqWheelCombH is saturated, reqWheelCombDH = 0
    if (St.reqWheelCombH > Pr.reqWheelCombHMax || St.reqWheelCombH < -Pr.reqWheelCombHMax)
        St.reqWheelCombDH = 0;
    end
    
    
    %detumble idea: set rws angular momentum (H) = - sat H
end