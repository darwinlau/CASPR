
classdef RayGridGeneration < handle

    properties (SetAccess = private)
        q_begin     % The lower bound on grid generalised coordinates
        q_end       % The upper bound on grid generalised coordinates
        q_initial   % the value of the variable regarding to the axis with zero discritization number 
        nsegvar     % the vector of discritization number each axes
        delta_q     % The step size in generalised coordinates
        q_length    % The length of each index
        listnflxvar % the varible of the axes with non-zero discritization number
        nflexvar    % number of variable with non-zero discritization number
    end

    methods
        % The constructor for the grid.
        function id = RayGridGeneration(q_begin,q_end,q_initial,nsegvar)
            CASPR_log.Assert((size(q_begin,2)==1)&&(size(q_end,2)==1)&&(size(nsegvar,2)==1),'Input to RayGridGeneration must be a column vector');
            CASPR_log.Assert((size(q_begin,1)==size(q_end,1))&&(size(q_begin,1)==size(nsegvar,1)),'Inputs must be of the same dimension');
            CASPR_log.Assert(isempty(find(((q_end - q_begin)<0),1)),'Invalid Input Range');
            CASPR_log.Assert(isempty(find(nsegvar<0)),'Invalid Input Range');
            % Maybe add more checks to ensure           
            id.q_begin  =   q_begin;
            id.q_end    =   q_end;
            id.q_initial= q_initial;
            id.nsegvar  =   nsegvar;           
            nvar=(size(q_begin,1));          
            for itvar=1:nvar
                if (id.nsegvar(itvar)==0)|| (id.q_end(itvar)-id.q_begin(itvar)==0)      
                    id.delta_q(itvar,1)=0;
                    id.nsegvar(itvar,1)=0;
                    id.q_begin(itvar,1)=q_initial(itvar);
                    id.q_end(itvar,1)=q_initial(itvar);
                else
                    id.delta_q(itvar,1)= (id.q_end(itvar)-id.q_begin(itvar))/nsegvar(itvar);
                end
            end
            id.q_length=id.nsegvar+1;  
            
            id.listnflxvar=linspace(1,nvar,nvar)';
            [zerorow,~]=find(id.nsegvar==0);
            for itzirid=1:length(zerorow)
                id.listnflxvar(id.listnflxvar==zerorow(itzirid))=[];
            end
            id.nflexvar=length(id.listnflxvar); 
        end
        
        
        
        function obj=DimensionReduction(obj,itnflexvar)
            CASPR_log.Assert(itnflexvar<=obj.nflexvar,'eceeds the number of flexible variables');
            
            curflexvar=obj.listnflxvar(itnflexvar);
            obj.q_begin(curflexvar,:)=[];
            obj.q_end(curflexvar,:)=[];
            obj.q_initial(curflexvar,:)=[];
            obj.nsegvar(curflexvar,:)=[];
            obj.delta_q(curflexvar,:)=[];     
            obj.q_length(curflexvar,:)=[];    
%             obj.listnflxvar(curflexvar,:)=[];
%             obj.nflexvar=obj.nflexvar-1;
        end

        function cursegvar=nod2vect(obj,nodlist)  %nsegvar= the division number of interval of variables
            nodlist=nodlist-1;                 % a list of number of nodes for which the vector
            % of number of segment of each variable is computed
            nvar=length(obj.nsegvar);
            [nodnum,~]=size(nodlist);      %just to find the number of current variables
            weight=prod((obj.nsegvar(1:nvar)+1));  % weight  of binary to decimal
            
            for itnvar=nvar:-1:1%:nvar
                weight=weight/(obj.nsegvar(itnvar)+1);
                cursegvar(1:nodnum,itnvar)=fix(nodlist/weight);
                nodlist=rem(nodlist,weight);
            end
        end  
        
        function nodlist=vect2nod(obj,cursegvar)   %nsegvar= the division number of interval of variables
            nvar=length(obj.nsegvar);
            [nodnum,~]=size(cursegvar);
            weight=1;
            nodlist=zeros(nodnum,1);
            for itnvar=1:nvar%:-1:1
                nodlist=cursegvar(:,itnvar)*weight+nodlist;
                weight=(obj.nsegvar(itnvar)+1)*weight;
            end
            nodlist=nodlist+1;
        end   
        
        function matsegvar=var2vect(obj,listvar)
            [nrow,ncol]=size(listvar);
            matsegvar=zeros(nrow,ncol);
            colzdel=find(obj.delta_q~=0);
            matsegvar(:,colzdel)=round((listvar(:,colzdel)-ones(nrow,1)*obj.q_begin(colzdel,1)')./(ones(nrow,1)*obj.delta_q(colzdel,1)'));
        end
        
        % Obtain the grid point from a given index
        function q = getGridPoint(obj,cursegvar)
            q=obj.q_begin'+obj.delta_q'.*cursegvar;       %the current magnitude of the variable which are constant
        end
    end
end
