
function matunion=fununion(mat)
if ~isempty(mat)
    [nrow,~]=size(mat);
    [fcolsor,indsort]=sort(mat,1);
    mat=mat(indsort(:,1),:);
    itrow=1;
    while itrow<nrow
        if mat(itrow+1,1)<=mat(itrow,2)
            if mat(itrow,2)<mat(itrow+1,2)
                mat(itrow,2)=mat(itrow+1,2);
            end
            mat(itrow+1,:)=[];
            nrow=nrow-1;
        else
            itrow=itrow+1;
        end
    end
    matunion=mat;
else
    matunion=[];
end



