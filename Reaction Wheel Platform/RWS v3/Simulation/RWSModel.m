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
        St = updateStateModel(St,Pr);
        Datalog = updateDatalog(St,Datalog,iteration);
    end
    plotDatalog(Pr,Datalog);
end

function Pr = initParams() % things that don't change
    Pr.satThStart = angle2quat(0,0,0);	% initial heading angle of RWS (quaternion)
    Pr.satWStart = [-0.5,2.5,3.0];             % initial angular velocity of RWS
    Pr.motorWsStart = [0,0,0,0];        % initial motor speeds (1x scalar per motor)
    Pr.motorI = 1.41E-05;               % mass moment of inertia of motor + wheel
    Pr.satI = 2.2E-03;                	% mass moment of inertia of 1U satellite (approximated as a uniform 1.33kg 10cm^3 cube)
    Pr.simDt = 0.001;                     % simulation time step (s)
    Pr.simDuration = 5;                 % length of simulation (s)
    Pr.motorWMax = 6700/60*360/180*pi;  % 2610T006B SC motor max speed
    Pr.targetTh = 90/180*pi;
    %implied parameters
    Pr.simIters = Pr.simDuration / Pr.simDt;    % number of iterations (calculated)
end

function St = initStateModel(Pr) %things that change each cycle
    St.satTh = Pr.satThStart;
    St.satW = Pr.satWStart;
    St.motorWs = Pr.motorWsStart;
    St.motorDH = [0,0,0];
    St.motorW = [0,0,0];  %testing
    St.errorPrev = 0;
    St.errorCumul = 0;
end

function Datalog = initDatalog(Pr)
    Datalog.satTh = zeros(Pr.simIters,4);
    Datalog.satW = zeros(Pr.simIters,3);
    Datalog.motorWs = zeros(Pr.simIters,4);
end

function Datalog = updateDatalog(St,Datalog,iteration)
    Datalog.satTh(iteration,:) = St.satTh;
    Datalog.satW(iteration,:) = St.satW;
    Datalog.motorWs(iteration,:) = St.motorWs;
end

function St = detumbleController(St,Pr)
    % simple P controller
    error = 0-St.satW(3);
    P = -0.001; %with Pr.simDt = 0.001
    St.motorDH = P*error;
    
    % motor can only change H as fast as torque*dt
    torqueMax = 6*0.001*Pr.simDt;
    St.motorDH = min(St.motorDH, torqueMax);
    St.motorDH = max(St.motorDH, -torqueMax);
        % maths:
        % T = I*alpha
        % dw = alpha*dt
        % H = I*w
        % dH = I*dw
        % dH = T*dt
    
    % if motorW is saturated, motorDH = 0
    if (St.motorW > Pr.motorWMax || St.motorW < -Pr.motorWMax)
        St.motorDH = 0;
    end
    
    
    %detumble idea: set rws angular momentum (H) = - sat H
end



function St = pointController(St,Pr)
    % simple P controller
    theta = quat2eul(St.satTh);
    error = Pr.targetTh-theta(1);
    P = -0.1;
    I = -0.0000001;
    D = -0.04;
    dError = (error - St.errorPrev)/Pr.simDt;
    St.errorPrev = error;
    St.errorCumul = St.errorCumul + error*Pr.simDt;
    St.motorDH = P*error + I*St.errorCumul + D*dError;
    
    %normalise for sim dT
    St.motorDH = St.motorDH*Pr.simDt;


    %a = St.motorDH
    % motor can only change H as fast as torque*dt
    dHMax = 6*0.001*Pr.simDt;
    St.motorDH = min(St.motorDH, dHMax);
    St.motorDH = max(St.motorDH, -dHMax);
    

        
    % if motorW is going to be saturated, motorDH = 0
    motorWNext = St.motorW + St.motorDH/Pr.motorI;
    if (motorWNext > Pr.motorWMax || motorWNext < -Pr.motorWMax)
        St.motorDH = 0;
    end

    
    %detumble idea: set rws angular momentum (H) = - sat H
end

function St = detumbleControllerQuat(St,Pr)
    % simple P controller
    error = 0-St.satW;
    P = 0.001; %with Pr.simDt = 0.001
    St.motorDH = P*error;
    
    %normalise for dT >to do?
    
    % motor can only change H as fast as torque*dt
    torqueMax = 6*0.001*Pr.simDt;
    St.motorDH = min(St.motorDH, torqueMax);
    St.motorDH = max(St.motorDH, -torqueMax);
        % maths:
        % T = I*alpha
        % dw = alpha*dt
        % H = I*w
        % dH = I*dw
        % dH = T*dt
    
    % if motorW is saturated, motorDH = 0
    if (max(St.motorW) > Pr.motorWMax || min(St.motorW) < -Pr.motorWMax)
        St.motorDH = 0;
    end
    
    
    %detumble idea: set rws angular momentum (H) = - sat H
end


function St = pointControllerQuat(St,Pr)
    % errorquat = quatmultiply(targetquat, inverse(currentquat))
    quatnormalize(St.satTh);
    errorQuat = quatmultiply([1 0 0 0],quatconj(St.satTh));
    errorDir = errorQuat(2:4);
    
    %{
    motorDH in direction of error, magnitude theta (2arccos(errorQuat))
    
    3x PIDs: 1 azimuth 1 elevation 1 angle (axis-angle)?
    3x PIDs for each axis?
    Some multi-input-multi-output controller?
    
    %}

    % errorvect = errorquat(2:4)
    % output = P*errorvect
    %sourcetree when done
    
    P = 0.1;
    I = 0.0000001;
    D = 0.04;
    dError = (error - St.errorPrev)/Pr.simDt;
    St.errorPrev = error;
    St.errorCumul = St.errorCumul + error*Pr.simDt;
    St.motorDH = P*error + I*St.errorCumul + D*dError;
    
    %normalise for sim dT
    St.motorDH = St.motorDH*Pr.simDt;

end

function St = updateStateModel(St,Pr)
    % calculate alpha
    %alpha = torque./Pr.satI;
    
    % calculate w
    %St.satW = St.satW + alpha * Pr.simDt;
    %St.satW = St.satW + [0,0,St.motorDH/Pr.satI];
    St.satW = St.satW + St.motorDH./Pr.satI;
    
    % calculate dTheta (quat version of dTheta = w*dt)
    dThNr = quatnormalize([1,St.satW * Pr.simDt *0.5]);
    
    % add dTheta to Theta
    St.satTh = quatmultiply(St.satTh, dThNr);
    
    % add motordW to motorW
    St.motorW = St.motorW + St.motorDH/Pr.motorI;
end

function plotDatalog(Pr,Datalog)
    figure(1);
    % graph sat w (roll pitch yaw)
    subplot(2,2,1);
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
    subplot(2,2,3);
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
    subplot(2,2,2);
    t = linspace(0,Pr.simDuration, Pr.simIters);
    satTh = quat2eul(Datalog.satTh)*180/pi;
    hold off;
    plot(t,90-satTh(:,1),'color', [0.8 0.8 0]); %yaw
    hold on;
    plot(t,satTh(:,2),'m');                 %pitch
    plot(t,satTh(:,3),'r');                 %roll
    title('Sat Th');
    axis([0,Pr.simDuration,-180,180]);
    
    
    % graph heading angle over time
    subplot(2,2,4);
    %+ve yaw is anticlockwise??
    nth = 50;
    vrot = quatrotate(Datalog.satTh, [1,0,0]);
    arrowX = vrot(:,1);
    arrowY = vrot(:,2);
    arrowZ = vrot(:,3);
    Os = zeros(Pr.simIters,1);
    quiver3(Os(nth:nth:end),Os(nth:nth:end),Os(nth:nth:end),arrowX(nth:nth:end),arrowY(nth:nth:end),arrowZ(nth:nth:end),1);
    title('RW-ADCS Heading');
    axis([-1,1,-1,1,-1,1]);
    hold on;
    

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
