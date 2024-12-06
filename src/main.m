%% Template Assignment 2 - Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); % put relevant functions and classes inside the /include folder 

%% Compute the geometric model for the given manipulator
iTj_0 = BuildTree();
disp('iTj_0')
disp(iTj_0);
jointType = [0 0 0 0 0 1 0]; % specify two possible link type: Rotational, Prismatic.
geometricModel = geometricModel(iTj_0,jointType);

%% Given the following configurations compute the Direct Geometry for the manipulator

% Compute iTj : transformation between the base of the joint <i>
% and its end-effector taking into account the actual rotation/traslation of the joint
q = [pi/4, -pi/4, 0, -pi/4, 0, 0.15, pi/4];
geometricModel.updateDirectGeometry(q)
disp('iTj')
disp(geometricModel.iTj);

% Compute the transformation of the ee w.r.t. the robot base
bTe = geometricModel.getTransformWrtBase(length(jointType));  
disp('bTe')
disp(bTe)

bTfive = geometricModel.getTransformWrtBase(5);
bTthree = geometricModel.getTransformWrtBase(3);
threeTb = inv(bTthree);
threeTfive = bTfive * threeTb;
disp("3T5")
disp(threeTfive)

%% Given the previous joint configuration compute the Jacobian matrix of the manipulator
km = kinematicModel(geometricModel);
km.updateJacobian()
disp('Jacobian Matrix for the given q')
disp(km.J)

% Show simulation ?
show_simulation = true;

% Set initial and final joint positions
qi = q;
qf = [pi/4+pi/6, -pi/4, 0, -pi/4, 0, 0.15, pi/4];

%%%%%%%%%%%%% SIMULATION LOOP %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Simulation variables
% simulation time definition 
samples = 100;
t_start = 0.0;
t_end = 10.0;
t_count = (t_end-t_start)/samples;
t = t_start:t_count:t_end; 

figure
grid on 
hold on
title('MOTION OF THE MANIPULATOR')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
az = 48;
el = 25;
view(az,el)
cindex = 1;
csize = length(t);
cmap = colormap(parula(csize));
color = cmap(mod(cindex,csize)+1,:);

qSteps =[linspace(qi(1),qf(1),samples)', ...
    linspace(qi(2),qf(2),samples)', ...
    linspace(qi(3),qf(3),samples)', ...
    linspace(qi(4),qf(4),samples)', ...
    linspace(qi(5),qf(5),samples)', ...
    linspace(qi(6),qf(6),samples)', ...
    linspace(qi(7),qf(7),samples)'];

% LOOP 
for i = 1:samples

    brij= zeros(3,geometricModel.jointNumber);
    q = qSteps(i,1:geometricModel.jointNumber)';
    % Updating transformation matrices for the new configuration 
    geometricModel.updateDirectGeometry(q)
    % Get the transformation matrix from base to the tool
    bTe = geometricModel.getTransformWrtBase(length(jointType)); 

    %% ... Plot the motion of the robot 
    if (rem(i,0.1) == 0) % only every 0.1 sec
        
        for j=1:geometricModel.jointNumber
            bTi(:,:,j) = geometricModel.getTransformWrtBase(j); 
        end
    
        bri(:,1) = [0; 0; 0];
    
        for j = 1:geometricModel.jointNumber
            bri(:,j+1) = bTi(1:3,4,j);
    
        end
    
        for j = 1:geometricModel.jointNumber+1
            plot3(bri(1,j), bri(2,j), bri(3,j),'bo')
            
        end
    
        color = cmap(mod(cindex,csize)+1,:);
        cindex = cindex + 1;
    
        line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', color)
    end


    if show_simulation == true
        drawnow
    end    

end

%%Plot the final configuration of the robot
figure
grid on 
hold on
title('FINAL CONFIGURATION')
xlabel('x')
ylabel('y')
zlabel('z')
axis equal
az = 48;
el = 25;
view(az,el)
cindex = 1;
for j = 1:geometricModel.jointNumber+1
    plot3(bri(1,j), bri(2,j), bri(3,j),'bo')
    
end

color = cmap(mod(cindex,csize)+1,:);
cindex = cindex + 1;
line(bri(1,:), bri(2,:), bri(3,:), 'LineWidth', 1.5, 'Color', color)
view(gca(),[90 0]);
