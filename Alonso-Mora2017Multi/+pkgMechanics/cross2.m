% calculate the cross product of vec1 and vec2 (scalar means in z axis)
function outputVector = cross2(inputVector1,inputVector2)
        RotMatCross = [0,-1;1,0];
        switch size(inputVector1,1)+size(inputVector2,1)
            case 2
                outputVector = zeros(1,size(inputVector1,2));
            case 3
                if size(inputVector1,1)==2
                    inputVector1Cross = -RotMatCross*inputVector1;
                    outputVector = inputVector1Cross.*inputVector2;
                else
                    inputVector2Cross = -RotMatCross*inputVector2;
                    outputVector = -inputVector2Cross.*inputVector1;
                end
            case 4 
                inputVector1Cross = RotMatCross*inputVector1;
                outputVector = diag(inputVector1Cross'*inputVector2)';
            otherwise
                error('Varargin Error: Input dimension should be either 1xN or 2xN.')
        end
end