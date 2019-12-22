% Mass Point Planar Model class (2 inputs: inputInertia(1x1),inputPosition(2x1))
% (last mod.: 27-03-2019, Author: Chu Wu)
% Methods:
% - getStates
% - updateStates
classdef MassPointPlanar < pkgMechanics.BodyConfig
    methods
        function obj = MassPointPlanar(inputInertia,inputPosition) 
            if nargin==0
                % default initiation
                superargs = {1,[0;0]};
            else
                if nargin>=2
                    if isscalar(inputInertia)&&isequal(size(inputPosition),[2,1])
                        % you can also use size(inputPosition,1) to compare
                        % the number of rows (size(inputPosition,2) for columns)
                        superargs = {inputInertia,inputPosition};
                    else
                        error('Varargin Error: Input size should be 1x1(inputInertia) and 2x1(inputPosition).')
                    end
                else
                    error('Nargin Error: Not enough input arguments.');
                end 
            end
            obj = obj@pkgMechanics.BodyConfig(superargs{:});
        end
        
        function getStates(obj)
            disp('------MassPointPlanar------')
            disp(['mass: ',num2str(obj.inertia)])
            disp(['position: ',pkgMechanics.mat2strDisplay(obj.position,2)])
            disp(['velocity: ',pkgMechanics.mat2strDisplay(obj.velocity,2)])
            disp('---------------------------')
        end
        
        function updateStates(obj,force,cycle)
            translationAccelaration = force/obj.inertia;
            obj.position = obj.position+obj.velocity*cycle+translationAccelaration*cycle^2/2;
            obj.velocity = obj.velocity + translationAccelaration*cycle;
        end  
    end
end