



classdef WorkspaceRayGeneration < handle
    
    
    properties
        
        model
        grid            % Grid object for brute force workspace (input)
        MatRays
        MatNodeGrid
        numRays
    end
    
    
    methods
        % The constructor for the workspace simulator class.
        function w = WorkspaceRayGeneration(model,grid)
            w.model         = model;
            w.grid          = grid;
            
        end
        
        
        function run(obj,varargin)
            
            if (nargin<2)
                min_ray_percent=0;
                ReadMode=0;
            elseif (nargin<3)
                min_ray_percent=varargin{1};
                ReadMode=0;
            else
                min_ray_percent=varargin{1};
                ReadMode=varargin{2};
            end
            numDofs=obj.model.numDofs;
            itwritex1=1000;                                %iteration-write-text
            textndigit='%12.12f';
            newmatseglin=[];                               %matrix of segment line       --matrix-segment-line
            matseglin=[];                               %matrix of segment line       --matrix-segment-line
            matnod=[];                               %matrix of segment line       --matrix-segment-line
            
            itnflexvar=1;
            divitconsvar=1;
            countlin=0;

            
            if ReadMode==0               
                
                dlmwrite([CASPR_configuration.LoadHomePath,'/WorkspaceRay/TempData/matseglin.txt'],matseglin);
                dlmwrite([CASPR_configuration.LoadHomePath,'/WorkspaceRay/TempData/matnod.txt'],matnod);
                while itnflexvar<=obj.grid.nflexvar
                    curflexvar=obj.grid.listnflxvar(itnflexvar);
                    CuruGrid= RayGridGeneration(obj.grid.q_begin,obj.grid.q_end,obj.grid.q_initial,obj.grid.nsegvar);
                    CuruGrid.DimensionReduction(itnflexvar);
                    maxitconsvar=prod(CuruGrid.q_length);
                    divconsvar=fix(maxitconsvar/itwritex1);
                    while divitconsvar <= divconsvar+1
                        sitconsvar=(divitconsvar-1)*itwritex1+1;
                        fitconsvar=(divitconsvar)*itwritex1;
                        if fitconsvar>maxitconsvar
                            fitconsvar=maxitconsvar;
                        end
                        for itconsvar=sitconsvar:fitconsvar
                            itconsvar;
                            cursegvar=CuruGrid.nod2vect(itconsvar);       %from nod2vect calculate the current segment of each variable---n=1--->>[0 0 0 ..]
                            magconvar=CuruGrid.getGridPoint(cursegvar);
                            admsrng=segment_computation(obj,curflexvar,magconvar,min_ray_percent);   %computing the range of the admissible range of last variable in wcw
                            [curnseglin spare]=size(admsrng);                                %number of segment line
                            if curnseglin>0
                                matseglin=[matseglin;[ones(curnseglin,1)*[curflexvar magconvar cursegvar] admsrng(:,:)]];
                            end
                        end
                        [nseglin spare]=size(matseglin);
                        for itnlin=1:nseglin
                            countlin=countlin+1;
                            lsegvarit=ceil((matseglin(itnlin,2*numDofs)-obj.grid.q_begin(curflexvar,1))/obj.grid.delta_q(curflexvar));                   %the lower segment number of the variable of the line segment
                            usegvarit=fix((matseglin(itnlin,2*numDofs+1)-obj.grid.q_begin(curflexvar,1))/obj.grid.delta_q(curflexvar));                 %the upper segment number of the variable of the line segment
                            matcursegvar=[ones(usegvarit-lsegvarit+1,1)*matseglin(itnlin,(numDofs+1):(numDofs+curflexvar-1)) (lsegvarit:usegvarit)' ones(usegvarit-lsegvarit+1,1)*matseglin(itnlin,(numDofs+curflexvar):(numDofs+numDofs-1))];
                            
                            
                            nodlist=obj.grid.vect2nod(matcursegvar)';
                            
                            countnod=length(nodlist);
                            newmatseglin(itnlin,1:numDofs+3+countnod)=[countlin matseglin(itnlin,1:numDofs) matseglin(itnlin,2*numDofs:2*numDofs+1) nodlist];
                            
                            curmatnod=[];
                            curmatnod(:,1)=nodlist';
                            curmatnod(:,2)=ones(countnod,1)*countlin;
                            matnod=[matnod;curmatnod];
                            
                            
                        end
                        
                        
                        dlmwrite([CASPR_configuration.LoadHomePath,'/WorkspaceRay/TempData/matseglin.txt'],newmatseglin,'precision',textndigit,'-append','delimiter',' ');
                        dlmwrite([CASPR_configuration.LoadHomePath,'/WorkspaceRay/TempData/matnod.txt'],matnod,'precision',textndigit,'-append','delimiter',' ');
                        
                        matseglin=[];
                        newmatseglin=[];
                        matnod=[];
                        
                        
                        divitconsvar=divitconsvar+1;
                        fitconsvar
                    end
                    divitconsvar=1;
                    itnflexvar=itnflexvar+1;
                end
            end
            
            fid=fopen('WorkspaceRay/TempData/matseglin.txt');
            obj.MatRays=[];
            obj.MatNodeGrid=[];
            obj.numRays=0;
            if fgetl(fid) == -1
                disp('The workspace is empty');
            else
                obj.MatRays=dlmread('WorkspaceRay/TempData/matseglin.txt');
                obj.MatNodeGrid=dlmread('WorkspaceRay/TempData/matnod.txt');
                [nrow ncol]=size(obj.MatRays);
                obj.numRays=nrow;
            end
        end
        
        function admsrng=segment_computation(obj,curflexvar,magconvar,min_ray_percent)
            zeroval=1e-8;
            admsrng=[];
            numDofs=obj.model.numDofs;
            numCables=obj.model.numCables;
            curtypevar= obj.model.bodyModel.q_dofType;
            %             curtypevar(curflexvar)=[];
            %             revjoint=find(curtypevar==1);
            %             multiconst=prod(tan(magconvar(revjoint)/2).^2+1);
            if curtypevar(curflexvar)==DoFType.TRANSLATION
                maxdeg=numDofs;
            else
                maxdeg=2*numDofs;
            end
            flxvarlinspace=linspace(obj.grid.q_begin(curflexvar),obj.grid.q_end(curflexvar),maxdeg+1);
            cab_comb=nchoosek(1:numCables,numDofs+1);
            [numcomb spar]=size(cab_comb);
            matf=zeros(maxdeg+1,numcomb*(numDofs+1));
            for it1=1:maxdeg+1
                magvar=[magconvar(1:curflexvar-1) flxvarlinspace(it1) magconvar(curflexvar:end)];
                obj.model.update(magvar', zeros(numDofs,1), zeros(numDofs,1),zeros(numDofs,1));
                if curtypevar(curflexvar)==DoFType.TRANSLATION
                    totmatdet=-(obj.model.L)';%multiconst*
                else
                    totmatdet=-(1+tan(flxvarlinspace(it1)/2)^2)*(obj.model.L)';%*multiconst
                end
                for itncable=1:obj.model.numCables
                    totmatdet(:,itncable)=totmatdet(:,itncable)*obj.model.cableLengths(itncable);
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
            for itcomb=1:numcomb
                matcoefpoly=[];
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
                finrealr=[obj.grid.q_begin(curflexvar),obj.grid.q_end(curflexvar)]';  %% final-real-roots
                for it=1:numDofs+1
                    curcoef=matcoefpoly(it,:);
                    curlencoef=length(curcoef);
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
                    if curtypevar(curflexvar)==DoFType.TRANSLATION
                        curcomr=curcomr;  % eliminating the complex roots
                    else
                        curcomr=2*atan(curcomr);  % eliminating the complex roots
                    end
                    finrealr=[finrealr;curcomr];       % storing the current real roots to the final-real-roots matrix
                end
                finrealr=finrealr';
                finrealr(finrealr<obj.grid.q_begin(curflexvar))=[];          %eliminating the roots beyond the bound of the variable
                finrealr(finrealr>obj.grid.q_end(curflexvar))=[];          %eliminating the roots beyond the bound of the variable
                finrealr=sort(finrealr);
                
                for it1=1:length(finrealr)-1
                    segpercent=((finrealr(1,it1+1)-finrealr(1,it1))/(obj.grid.q_end(curflexvar)-obj.grid.q_begin(curflexvar)))*100;
                    if segpercent>min_ray_percent
                        
                        if curtypevar(curflexvar)==DoFType.TRANSLATION
                            tstval=mean([finrealr(1,it1) finrealr(1,it1+1)]);
                        else
                            tstval=tan(mean([finrealr(1,it1) finrealr(1,it1+1)])/2);
                        end
                        
                        signvect=[];
                        for it2=1:numDofs+1
                            signvect(it2)=polyval(matcoefpoly(it2,:),tstval);
                        end
                        if (signvect>zeroval)
                            admsrng=[admsrng;[finrealr(it1) finrealr(it1+1)]];
                        end
                        if (signvect<-zeroval)
                            admsrng=[admsrng;[finrealr(it1) finrealr(it1+1)]];
                        end
                    end
                end
            end
            admsrng=fununion(admsrng);
        end
        
        
        
        
        function plotRayWorkspace(obj,plot_axis)   %nsegvar= the division number of interval of variables
            
            numDofs=obj.model.numDofs;
            if numDofs>2
                if(nargin<2)
                    plot_axis=[1,2,3];
                end
                axis1=[];
                axis2=[];
                axis3=[];
                for it=1:obj.numRays
                    plotflag=0;
                    consvar=[obj.MatRays(it,3:(3+obj.MatRays(it,2)-2)),0,obj.MatRays(it,(3+obj.MatRays(it,2)-1):(3+numDofs-2))];
                    if obj.MatRays(it,2)==plot_axis(1)
                        axis1=(obj.MatRays(it,numDofs+2:numDofs+3));
                        axis2=(consvar(plot_axis(2))*ones(1,2));
                        axis3=(consvar(plot_axis(3))*ones(1,2));
                        plotflag=1;
                    elseif obj.MatRays(it,2)==plot_axis(2)
                        axis1=(consvar(plot_axis(1))*ones(1,2));
                        axis2=(obj.MatRays(it,numDofs+2:numDofs+3));
                        axis3=(consvar(plot_axis(3))*ones(1,2));
                        plotflag=1;
                    elseif obj.MatRays(it,2)==plot_axis(3)
                        axis1=(consvar(plot_axis(1))*ones(1,2));
                        axis2=(consvar(plot_axis(2))*ones(1,2));
                        axis3=(obj.MatRays(it,numDofs+2:numDofs+3));
                        plotflag=1;
                    end
                    if plotflag==1
                        plot3(axis1,axis2,axis3,'k','LineWidth',1);
                        axis1=strcat('axis_',int2str(plot_axis(1)));
                        axis2=strcat('axis_',int2str(plot_axis(2)));
                        axis3=strcat('axis_',int2str(plot_axis(3)));
                        xlabel(axis1)
                        ylabel(axis2)
                        zlabel(axis3)
                        hold on
                    end
                end
            else
                if(nargin<2)
                    plot_axis=[1,2];
                end
                axis1=[];
                axis2=[];
                for it=1:obj.numRays
                    plotflag=0;
                    consvar=[obj.MatRays(it,3:(3+obj.MatRays(it,2)-2)),0,obj.MatRays(it,(3+obj.MatRays(it,2)-1):(3+numDofs-2))];
                    if obj.MatRays(it,2)==plot_axis(1)
                        axis1=(obj.MatRays(it,numDofs+2:numDofs+3));
                        axis2=(consvar(plot_axis(2))*ones(1,2));
                        plotflag=1;
                    elseif obj.MatRays(it,2)==plot_axis(2)
                        axis1=(consvar(plot_axis(1))*ones(1,2));
                        axis2=(obj.MatRays(it,numDofs+2:numDofs+3));
                        plotflag=1;
                    end
                    if plotflag==1
                        plot(axis1,axis2,'k','LineWidth',1);
                        axis1=strcat('axis_',int2str(plot_axis(1)));
                        axis2=strcat('axis_',int2str(plot_axis(2)));
                        xlabel(axis1)
                        ylabel(axis2)
                        hold on
                    end
                end
            end
        end
    end
end







