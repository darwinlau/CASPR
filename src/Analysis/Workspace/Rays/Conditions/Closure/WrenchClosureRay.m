% Class to compute whether a pose (dynamics) is within the wrench-closure
% workspace (WCW)
%
% Author        : Jonathan EDEN
% Created       : 2015
% Description    : 
classdef WrenchClosureRay < WorkspaceRayConditionBase
    properties (SetAccess = protected, GetAccess = protected)
        min_ray_percentage          % The minimum percentage of the ray at which it is included
    end
    
    methods
        % Constructor for wrench closure workspace
        function w = WrenchClosureRay(min_ray_percent)
            w.min_ray_percentage = min_ray_percent;
        end
        
        % Evaluate the wrench closure condition return true if satisfied 
        function intervals = evaluateFunction(obj,model,workspace_ray)
            zeroval=1e-8; % THIS COULD BE MADE AN OPTION FOR THE SOLVER
            % Variable initialisation
            numDofs=model.numDofs;
            numCables=model.numCables;
            curtypevar= model.bodyModel.q_dofType;
            curflexvar = workspace_ray.free_variable_index;
            % Use the joint type to determine the maximum polynomial degree
            if curtypevar(curflexvar)==DoFType.TRANSLATION
                maxdeg=numDofs;
            else
                maxdeg=2*numDofs;
            end
            % Set up a lin space for the free variable
            flxvarlinspace=linspace(workspace_ray.free_variable_range(1),workspace_ray.free_variable_range(2),maxdeg+1);
            % Determine combinatoric vaqiables
            cab_comb=nchoosek(1:numCables,numDofs+1);
            [numcomb,~]=size(cab_comb);
            % Compute matf
            matf=zeros(maxdeg+1,numcomb*(numDofs+1));
            % Set up the pose vector
            q_fixed = workspace_ray.fixed_variables;
            fixed_index = true(numDofs,1); fixed_index(workspace_ray.free_variable_index) = false;
            q = zeros(numDofs,1); q(fixed_index) = q_fixed;
            for it1=1:maxdeg+1
                q(workspace_ray.free_variable_index) = flxvarlinspace(it1);
                model.update(q, zeros(numDofs,1), zeros(numDofs,1),zeros(numDofs,1));
                if curtypevar(curflexvar)==DoFType.TRANSLATION
                    totmatdet=-(model.L)';%multiconst*
                else
                    totmatdet=-(1+tan(flxvarlinspace(it1)/2)^2)*(model.L)';%*multiconst
                end
                for itncable=1:numCables
                    totmatdet(:,itncable)=totmatdet(:,itncable)*model.cableLengths(itncable);
                end
                countcol=0;
                for it2=1:numcomb
                    matdet=totmatdet(:,cab_comb(it2,:));
                    for it3=1:numDofs+1
                        countcol=countcol+1;
                        curmatdet=matdet;    %%current-matrix-determinant
                        curmatdet(:,it3)=[];
                        matf(it1,countcol)=det(curmatdet);
                    end
                end
            end
            countcol=0;
            intervals=zeros(numcomb*((maxdeg+2)*numDofs),2);
            interval_i = 1;
            for itcomb=1:numcomb
                matcoefpoly=zeros(numDofs+1,maxdeg+1);
                for itdof=1:numDofs+1
                    countcol=countcol+1;
                    vectf=matf(:,countcol);
                    
                    if curtypevar(curflexvar)==DoFType.TRANSLATION
                        
                        coefpoly=((-1)^(itdof+1))*polyfit(flxvarlinspace,vectf',maxdeg);
                    else
                        
                        coefpoly=((-1)^(itdof+1))*polyfit(tan(flxvarlinspace/2),vectf',maxdeg);
                    end
                    matcoefpoly(itdof,:)=coefpoly;
                end
                % THE LINE BELOW SHOULD CHANGE (IN THAT IT IS INEFFICIENT) BUT I HAVE LEFT IT
                % CONSISTENT WITH ORIGINAL CODE FOR THE MOMENT
                finrealr=zeros(2+numDofs*maxdeg,1);
                finrealr([1,2])=[workspace_ray.free_variable_range(1);workspace_ray.free_variable_range(2)];  %% final-real-roots
                finrealr_index = 3;
                curlencoef = maxdeg+1;
                for it=1:numDofs+1
                    curcoef=matcoefpoly(it,:);
                    numz=0;
                    for itzcoef=1:curlencoef
                        if abs(curcoef(itzcoef))<1e-8
                            numz=numz+1;
                        else
                            break
                        end
                    end
                    curcoef(1:numz)=[];
                    curcomr=roots(curcoef);   %current-complex-roots
                    curcomr=curcomr(imag(curcomr)==0);  % eliminating the complex roots
                    if curtypevar(curflexvar)~=DoFType.TRANSLATION
                        curcomr=2*atan(curcomr);  % eliminating the complex roots
                    end
                    finrealr(finrealr_index:finrealr_index+length(curcomr)-1)=curcomr;       % storing the current real roots to the final-real-roots matrix
                    finrealr_index = finrealr_index + length(curcomr);
                end
                finrealr = finrealr(1:finrealr_index-1)';
                finrealr(finrealr<workspace_ray.free_variable_range(1))=[];          %eliminating the roots beyond the bound of the variable
                finrealr(finrealr>workspace_ray.free_variable_range(2))=[];          %eliminating the roots beyond the bound of the variable
                finrealr=sort(finrealr);
                for it1=1:length(finrealr)-1
                    segpercent=((finrealr(1,it1+1)-finrealr(1,it1))/(workspace_ray.free_variable_range(2)-workspace_ray.free_variable_range(1)))*100;
                    if segpercent>obj.min_ray_percentage
                        
                        if curtypevar(curflexvar)==DoFType.TRANSLATION
                            tstval=mean([finrealr(1,it1) finrealr(1,it1+1)]);
                        else
                            tstval=tan(mean([finrealr(1,it1) finrealr(1,it1+1)])/2);
                        end
                        
                        signvect=zeros(numDofs+1,1);
                        for it2=1:numDofs+1
                            signvect(it2)=polyval(matcoefpoly(it2,:),tstval);
                        end
                        if (signvect>zeroval)
                            intervals(interval_i,:)=[finrealr(it1) finrealr(it1+1)];
                            interval_i = interval_i + 1;
                        end
                        if (signvect<-zeroval)
                            intervals(interval_i,:)=[finrealr(it1) finrealr(it1+1)];
                            interval_i = interval_i + 1;
                        end
                    end
                end
            end
            intervals=obj.mat_union(intervals(1:interval_i-1,:));
        end
    end
    
    methods (Access = private)
        % THIS SHOULD PROBABLY BE MOVED ELSEWHERE
        function matunion=mat_union(obj,mat) %#ok<INUSL>
            if ~isempty(mat)
                [nrow,~]=size(mat);
                [~,indsort]=sort(mat,1);
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
        end
    end
end