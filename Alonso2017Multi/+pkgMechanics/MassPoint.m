% Mass Point 3D Model class (2 inputs: inputMass(1x1),inputPosition(3x1))
% (last mod.: 17-05-2019, Author: Chu Wu)
% properties
% - position
% - velocity
% - acceleration
% - mass
% Methods:
% - setVelocity
% - forwardDynamics
% - getStates
% - updateStates
classdef MassPoint < handle
    % If a class defines superclasses, all or none must be handle classes.
    properties (SetAccess = protected)
        % states (3 dim)
        position
        velocity = [0 0 0]';
        acceleration = [0 0 0]';
        % params (1 dim)
        mass
    end
    
    methods
        function obj = MassPoint(inputMass,inputPosition)
            if nargin==0
                % default initiation
                superargs = {1,[0 0 0]'};
            else
                if nargin>=2
                    if isscalar(inputMass)&&isvector(inputPosition)&&length(inputPosition)==3
                        % you can also use size(inputPosition,1) to compare
                        % the number of rows (size(inputPosition,2) for columns)
                        if inputMass~=0
                            superargs = {inputMass,inputPosition(:)};
                        else
                            error('Imporper input value.')
                        end
                    else
                        error('Improper input dimension.')
                    end
                else
                    error('Not enough input arguments.');
                end
            end
            obj.mass = superargs{1};
            obj.position = superargs{2};
        end
        
        function obj = setVelocity(obj,inputVelocity)
            % if you want to use the method like object.method this way
            % then put the object into the input of the method
            if isvector(inputVelocity)&&length(inputVelocity)==3
                obj.velocity = inputVelocity(:);
            else
                error('Improper input dimension.')
            end
        end
        
        function obj = forwardDynamics(obj,force,ImpMass)
            if nargin == 3
                obj.acceleration = force/(obj.mass+ImpMass);
            else
                obj.acceleration = force/(obj.mass);
            end
        end
        
        function getStates(obj)
            import pkgMechanics.*
            disp('--------Mass Point---------')
            disp(['mass: ',num2str(obj.mass)])
            disp(['position: ',mat2strf(obj.position','%0.2f')])
            disp(['velocity: ',mat2strf(obj.velocity','%0.2f')])
            disp(['acceleration: ',mat2strf(obj.velocity','%0.2f')])
            disp('---------------------------')
        end
        
        function updateStates(obj,cycle)
            % note that for handles there is no need for output
            % but for values it should be obj = func(obj,input)
            obj.position = obj.position+obj.velocity*cycle+obj.acceleration*cycle^2/2;
            obj.velocity = obj.velocity + obj.acceleration*cycle;
        end
    end
end