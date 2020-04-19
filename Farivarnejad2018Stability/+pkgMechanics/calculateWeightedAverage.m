function weightedAverage = calculateWeightedAverage(inputWeightList,inputVariableList)
    if size(inputWeightList,1)==1&&size(inputVariableList,2)==size(inputWeightList,2)
        weightedAverage = sum(inputWeightList.*inputVariableList,2)/sum(inputWeightList);
    else
       error('Varargin Error: Input dimension should be inputWeightList(1xN),inputVariableList(mxN).')
    end
end