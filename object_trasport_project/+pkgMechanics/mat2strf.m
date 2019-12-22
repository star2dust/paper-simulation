% display a matrix in a format of %0.mf
function outStrMat = mat2strf(inNumMat,dispFmt)
[nrow,ncol] = size(inNumMat);
outStrMat = '[';
for i=1:nrow
    for j=1:ncol
        if j~=ncol
            outStrMat = [outStrMat,sprintf(dispFmt,inNumMat(i,j)),','];
        else
            if nrow~=1
                outStrMat = [outStrMat,sprintf(dispFmt,inNumMat(i,j)),';'];
            else
                outStrMat = [outStrMat,sprintf(dispFmt,inNumMat(i,j))];
            end
        end
    end
end
outStrMat = [outStrMat,']'];
end