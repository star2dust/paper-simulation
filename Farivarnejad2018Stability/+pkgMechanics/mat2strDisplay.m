function outStrMat = mat2strDisplay(inNumMat,inNumFmt)
[nrow,ncol] = size(inNumMat);
dispFmt = ['%0.',num2str(inNumFmt),'f'];
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