% Body Configure class (2 inputs: inputInertia(1x1),inputPosition(mx1))
% (last mod.: 27-03-2019, Author: Chu Wu)
% Properties:
% - inertia: inertia
% - states: position, velocity
classdef BodyConfig < handle
    properties
        % inertia
        inertia;
        % states
        position;
        velocity;
    end
    methods
        function obj = BodyConfig(inputInertia,inputPosition)
            if nargin==0
                % default initiation with dimension 1
                obj.inertia = 0;
                obj.position = 0;
                obj.velocity = 0;
            else
                if nargin>=2
                    if isscalar(inputInertia)&&isvector(inputPosition)
                        obj.inertia = inputInertia;
                        obj.position = inputPosition(:);
                        obj.velocity = zeros(size(inputPosition(:)));
                    else
                        error('Varargin Error: Input size should be 1x1(inputInertia) and mx1(inputPosition).')
                    end
                else
                    error('Nargin Error: Not enough input arguments.');
                end
            end
        end
    end
end