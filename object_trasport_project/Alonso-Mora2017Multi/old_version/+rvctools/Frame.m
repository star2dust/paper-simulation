% Frame 3D class (1 input: inputTself(4x4)))
% (last mod.: 04-06-2019, Author: Chu Wu)
% This is a supplement for RVC tools (Requires Robotics Toolbox of Peter Corke
% http://petercorke.com/wordpress/toolboxes/robotics-toolbox)

classdef Frame < handle
    properties
        % self transformation in parent frame (4x4 matrix)
        Tself2parent
        % self transformation in parent frame (4x4 matrix, default as inertia frame)
        Tparent = eye(4)
    end
    
    properties (SetAccess = protected)     
        % children transformation in self frame (each 4x4 in a cell)
        children = []  
    end
    
    methods
        function obj = Frame(inputTself)
            if size(inputTself)==[4,4]
                obj.Tself2parent = inputTself;
            else
                error('Improper input dimension.')
            end
        end
        
        function obj = addChild(obj,inputTs2c,inputName)
            seq = length(obj.children);
            if ischar(inputName)
                obj.children(seq+1).name = inputName;
            else
                error('Improper name.')
            end
            if size(inputTs2c)==[4,4]
                obj.children(seq+1).Ts2c = inputTs2c;
            else
                error('Improper input dimension.')
            end
        end
        
        function obj = delChild(obj,inputName)
            if ischar(inputName)
                logidx = contains({obj.children.name},inputName);
                obj.children(logidx) = [];
            else
                error('Improper name.')
            end
        end
    end
end