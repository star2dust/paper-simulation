% Rigid Body Planar Model class (2 inputs: inputMassList(1xN),inputPositionList(2xN))
% (last mod.: 27-03-2019, Author: Chu Wu)
% Properties:
% - states: translation, rotation
% - vertice: verticeList (polygon)
% Methods:
% - getStates
% - updateStates
classdef RigidBodyPlanar < handle
    properties
        % translation & rotation states < BodyConfig
        translation
        rotation
        % a list of mass points that make up a rigid body (1xN)
        verticeList = pkgMechanics.MassPointPlanar
        % relative position list (2xN) in rigid body frame
        relativePositionList
        % rotation matrix (2x2)
        rotationMatrixPlanar
    end
    
    properties (Transient)
        % statesListener property is transient so the listener handle is not saved.
        statesListener
    end
    
    events
       statesUpdated 
    end
    
    methods
        function obj = RigidBodyPlanar(inputMassList,inputPositionList)
            % basic configuration
            if nargin>=2
                if size(inputMassList,1)==1&&size(inputPositionList,1)==2
                    obj.generateVerticeList(inputMassList,inputPositionList);
                    % weighted average (sum(weighList.*variableList,2)/sum(weighList))
                    translationMass = sum(inputMassList);
                    translationPosition = sum(inputMassList.*inputPositionList,2)/sum(inputMassList);
                    obj.translation = pkgMechanics.BodyConfig(translationMass,translationPosition);
                    % parallel axis theorem (sum(weighList.*diag(variableList'*variableList)'))
                    obj.relativePositionList = [obj.verticeList.position]-obj.translation.position;
                    rotationMoment = sum(inputMassList.*diag(obj.relativePositionList'*obj.relativePositionList)');
                    obj.rotation = pkgMechanics.BodyConfig(rotationMoment,0);
                else
                    error('Varargin Error: Input dimension should be inputMassList(1xN),inputPositionList(2xN).')
                end
            else
                if nargin>0
                    error('Nargin Error: Not enough input arguments.');
                else
                    % default initiation
                    obj.translation = pkgMechanics.BodyConfig(1,[0;0]);
                    obj.rotation = pkgMechanics.BodyConfig(1,0);
                    obj.relativePositionList = [0;0];
                end
            end
            % addition configuration
            obj.generateRotationMatrixPlanar();
            obj.statesListener = addlistener(obj, 'statesUpdated', @(src,~)obj.onStatesUpdated(src));
        end
        
        function updateStates(obj,inputForceList,cycle)
            % class list ==> properity list
            positionList = [obj.verticeList.position];
            if isequal(size(inputForceList),size(positionList))
                % sum of forces and torques
                forceSum = sum(inputForceList,2);
                torqueSum = sum(pkgMechanics.crossProductPlanar(obj.relativePositionList,inputForceList));
                % translation
                translationAccelaration = forceSum/obj.translation.inertia;
                obj.translation.position = obj.translation.position + obj.translation.velocity*cycle + translationAccelaration*cycle^2/2;
                obj.translation.velocity = obj.translation.velocity + translationAccelaration*cycle;
                % rotation
                rotationAccelaration = torqueSum/obj.rotation.inertia;
                obj.rotation.position = obj.rotation.position + obj.rotation.velocity*cycle + rotationAccelaration*cycle^2/2;
                obj.rotation.velocity = obj.rotation.velocity + rotationAccelaration*cycle;
                % after position update is finished
                notify(obj,'statesUpdated')
            else
                error('The force input should be a 2xN matrix.')
            end
        end
            
        function getStates(obj)
            disp('-----RigidBodyPlanar------')
            disp('-------translation--------')
            disp(['mass: ',num2str(obj.translation.inertia)])
            disp(['position: ',pkgMechanics.mat2strDisplay(obj.translation.position,2)])
            disp(['velocity: ',pkgMechanics.mat2strDisplay(obj.translation.velocity,2)])
            disp(['vertices: ',pkgMechanics.mat2strDisplay([obj.verticeList.position],2)])
            disp(['relative positions: ',pkgMechanics.mat2strDisplay(obj.relativePositionList,2)])
            disp('--------rotation----------')
            disp(['moment: ',num2str(obj.rotation.inertia)])
            disp(['orientation: ',pkgMechanics.mat2strDisplay(obj.rotation.position,2)])
            disp(['angular velocity: ',pkgMechanics.mat2strDisplay(obj.rotation.velocity,2)])
            disp(['rotation matrix: ',pkgMechanics.mat2strDisplay(obj.rotationMatrixPlanar,2)])
            disp('--------------------------')
        end
    end
    
    methods (Access = protected)
        function obj = generateVerticeList(obj,inputMassList,inputPositionList)
            nVertice = length(inputMassList);
            for i=1:nVertice
                obj.verticeList(i) = pkgMechanics.MassPointPlanar(inputMassList(:,i),inputPositionList(:,i));
            end
        end
        
        function obj = generateRotationMatrixPlanar(obj)
           obj.rotationMatrixPlanar = [cos(obj.rotation.position),-sin(obj.rotation.position);sin(obj.rotation.position),cos(obj.rotation.position)];
        end
        
        function onStatesUpdated(eventSrc,eventData)
            % eventData(also an object) is used when you notify it (notify(obj,'statesUpdated',eventData))
            eventSrc.generateRotationMatrixPlanar;
            for i=1:length(eventSrc.verticeList)
                eventSrc.verticeList(i).position = eventSrc.translation.position + eventSrc.rotationMatrixPlanar*eventSrc.relativePositionList(:,i);
                eventSrc.verticeList(i).velocity = eventSrc.translation.velocity + pkgMechanics.crossProductPlanar(eventSrc.rotation.velocity,eventSrc.rotationMatrixPlanar*eventSrc.relativePositionList(:,i));
            end  
        end
    end
end