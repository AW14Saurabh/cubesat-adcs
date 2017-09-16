% Reaction Wheel Module v3 Simulator
% Mark Yeo; mark.yeo@student.unsw.edu.au
% Last modified 2017/09/17

function RWSModel()
    clear;
    state = initSimState();
    param = initParams();
    datalog = initDatalog(param);
    
    for iteration = 1:param.simIters
        sensor = getSensorData(state,param);
        torque = controller(state,param,sensor);
        state = updateSimState(state,param,torque);
        datalog = updateDatalog(state,datalog,sensor,iteration);
    end
    plotDatalog(param,datalog);
end

function state = initSimState() %things that change each cycle
    state.theta = angle2quat(0,0,0);    %angle of RWS
    state.w = angle2quat(pi/4,0,0);     %w of RWS, yaw pitch roll
    state.wMotor = [0,0,0,0];           %w (scalar) of motors
end

function param = initParams() %things that don't change
    param.B = angle2quat(0,0,0);	%B around RWS (global frame)
    param.BMax = 0.01;   %calc      %Max B expected
    param.rAccel = 0.05;          	%distance of accel to RWS CoM
    param.IMotor = 10; %calc        %I of each motor + wheel
    param.I = 100;     %calc       	%I of RWS
    param.simDt = 0.01;              %simulation cycle time step
    % Sensor params, measured from MPU9250, sample time 0.01s
    param.gyroNoise = [0.0008564 0.0007952 0.0008797]; %stddev, radians
    param.gyroBias = [-0.0226 -0.0026 -0.0085]; %radians
    param.accelNoise = [0.0104 0.0101 0.0166];  %stddev, m/s/s
    param.magNoise = [0.7542 0.7046 0.6937];    %stddev, uT (mag usually around 50-55uT)
    param.simDuration = 10;         %length of simulation (s)
    param.simIters = param.simDuration / param.simDt;
end

function datalog = initDatalog(param)
    datalog.theta = zeros(param.simIters,4);
    datalog.w = zeros(param.simIters,4);
    datalog.wMotor = zeros(param.simIters,4);
    datalog.gyro = zeros(param.simIters,3);
    datalog.accel = zeros(param.simIters,3);
    datalog.mag = zeros(param.simIters,3);
end

function datalog = updateDatalog(state,datalog,sensor,iteration)
    datalog.theta(iteration,:) = state.theta;
    datalog.w(iteration,:) = state.w;
    datalog.wMotor(iteration,:) = state.wMotor;
    datalog.gyro(iteration,:) = sensor.gyro;
    %datalog.accel(iteration,:) = sensor.accel;
    %datalog.mag(iteration,:) = sensor.mag;
end

function sensor = getSensorData(state, param)
    rot = quat2eul(state.w);
    sensor.gyro(1) = rot(1) + param.gyroNoise(1).*randn(1) + param.gyroBias(1);
    sensor.gyro(2) = rot(2) + param.gyroNoise(2).*randn(1) + param.gyroBias(2);
    sensor.gyro(3) = rot(3) + param.gyroNoise(3).*randn(1) + param.gyroBias(3);
    %sensor.accel = state.w * param.rAccel; (F = r x w)
    %sensor.mag = state.theta . param.B * ; (how much of theta is in B direction)
end

function torque = controller(state,param,sensor)
    torque = 0;
end


function state = updateSimState(state,param,torque)
    %convert torque of motors to theta'' of RWS (quat)
    %
    
    %convert theta'' & dt to theta' of RWS (quat)
    dtQ = quatpower(state.w, param.simDt);
    state.theta = quatmultiply(state.theta, dtQ);
end

function plotDatalog(param,datalog)
    figure(1);
    
    %Roll Pitch Yaw graphs
    subplot(2,2,1);
    t = linspace(0,param.simDuration, param.simIters);
    eul = quat2eul(datalog.theta)*180/pi;
    gyro = datalog.gyro*180/pi;
    hold on;
    %plot(t,eul(:,1),'color', [0.8 0.8 0]); %yaw
    %plot(t,eul(:,2),'m'); %pitch
    %plot(t,eul(:,3),'r'); %roll
    plot(t,gyro(:,1),'color', [0.8 0.8 0]); %yaw
    plot(t,gyro(:,2),'m'); %pitch
    plot(t,gyro(:,3),'r'); %roll

    axis([0,param.simDuration,-180,180]);
    hold off;
    
    %RWS heading
    subplot(2,2,2);
    v = [1,0,0];
    q = datalog.theta;
    vrot = quatrotate(q, v);
    x = vrot(:,1);
    y = vrot(:,2);
    z = vrot(:,3);
    O = zeros(param.simIters,1);
    quiver3(O,O,O,x,y,z);
    
    %Animate RWS heading
    %{
    for i = 1:param.simIters
        % plot line
        %plot3(x(1:i),y(1:i),z(1:i));
        % plot all
        %quiver3(zeros(i,1),zeros(i,1),zeros(i,1),x(1:i),y(1:i),z(1:i),1);
        % or
        hold on;
        quiver3(0,0,0,x(i),y(i),z(i),0.9);
        hold off;
        axis([-1,1,-1,1,-1,1]);
        drawnow
        pause(param.simDt/10)
    end
    %}
end
%Kalman
%https://au.mathworks.com/help/control/ug/kalman-filtering.html?requestedDomain=www.mathworks.com


%Simulink?
%https://au.mathworks.com/help/ident/examples/building-and-estimating-process-models-using-system-identification-toolbox.html
%https://au.mathworks.com/help/simulink/ug/modeling-dynamic-systems.html
%https://au.mathworks.com/help/ident/ug/what-is-a-process-model.html
