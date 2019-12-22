function moment=calculateMomentByParallelAxisThm(inputMassList,inputRelativePositionList)
if size(inputMassList,1)==1&&size(inputRelativePositionList,2)==size(inputMassList,2)
    moment = sum(inputMassList.*diag(inputRelativePositionList'*inputRelativePositionList)');
else
    error('Varargin Error: Input dimension should be inputMassList(1xN),inputRelativePositionList(mxN).')
end