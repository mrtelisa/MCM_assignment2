%% Kinematic Model Class - GRAAL Lab
classdef kinematicModel < handle
    % KinematicModel contains an object of class GeometricModel
    % gm is a geometric model (see class geometricModel.m)
    properties
        gm % An instance of GeometricModel
        J % Jacobian
    end

    methods
        % Constructor to initialize the geomModel property
        function self = kinematicModel(gm)
            if nargin > 0
                self.gm = gm;
                self.J = zeros(6, self.gm.jointNumber);
            else
                error('Not enough input arguments (geometricModel)')
            end
        end
        function updateJacobian(self)
        %% Update Jacobian function
        % The function update:
        % - J: end-effector jacobian matrix

            % TO DO
            self.J = zeros(6, self.gm.jointNumber);
            iTn = eye(4);
            ki = [0, 0, 1];
            for i = self.gm.jointNumber:-1:1
                iTn = self.gm.iTj(:,:,i) * iTn;
                irn = iTn(1:3, 4);
                if self.gm.jointType(i) == 0
                    self.J(:, i) = [ki, cross([0, 0, 1], irn)];
                else
                    self.J(:, i) = [zeros(1, 3), ki];
                end  
            end
        end
    end
end

