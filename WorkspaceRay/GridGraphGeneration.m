



classdef GridGraphGeneration < handle
    
    
    properties
        
        model
        grid
        workspace            % Grid object for brute force workspace (input)
        
        final_network
        uniq_line
        final_graph
    end
    
    
    methods
        
        function w = GridGraphGeneration(model,grid,workspace)
            w.model         = model;
            w.workspace     = workspace;
            w.grid          = grid;
        end
        
        
        function run(obj,varargin)
            
            if (nargin<2)
                WeightCond=0;
                ReadMode=0;
            elseif (nargin<3)
                WeightCond=varargin{1};
                ReadMode=0;
            else
                WeightCond=varargin{1};
                ReadMode=varargin{2};
            end
            
            
            itwritex=1000;
            textndigit='%12.12f';
            
            
            matseglin=[];
            matseglin=obj.workspace.MatRays;
            matnod=[];
            matnod=obj.workspace.MatNodeGrid;
            
            [sortcol,nsortcol]=sort(matnod(:,2));
            [uniqlin,nuniqlin]=unique(sortcol);
            nuniqlin=[nuniqlin;length(sortcol)+1]; %number-unique-node


            
            if  ReadMode==0
                matnetgrid=[];
                dlmwrite('WorkspaceRay/TempData/matnetgrid.txt',matnetgrid);
                lengnuniqlin=length(nuniqlin);
                for itunilin=1:lengnuniqlin-1
                    ncurcongrid=matnod(nsortcol(nuniqlin(itunilin):nuniqlin(itunilin+1)-1),1);  %number-current-connected-lines
                    
                    lengcurnet=length(ncurcongrid);
                    
                    
                    for itcurnet=1:lengcurnet-1
                        segvar1=obj.grid.nod2vect(ncurcongrid(itcurnet));                       
                        magvar1=obj.grid.getGridPoint(segvar1);
                        segvar2=obj.grid.nod2vect(ncurcongrid(itcurnet+1));
                        magvar2=obj.grid.getGridPoint(segvar2);
                        if WeightCond==1
                            inv_TF=TF_Index(obj,magvar1,magvar2);
                        else
                            inv_TF=1;
                        end
                       
                        matnetgrid=[matnetgrid;[ncurcongrid(itcurnet) ncurcongrid(itcurnet+1) inv_TF]];
                        
                    end
                    
                    if mod(itunilin,itwritex)==0
                        itunilin
                        matnetgrid=unique([sort(matnetgrid(:,1:2),2),matnetgrid(:,3)],'rows') ;
                        dlmwrite('WorkspaceRay/TempData/matnetgrid.txt',matnetgrid,'precision',textndigit,'-append','delimiter',' ');
                        matnetgrid=[];
                    end

                end
                matnetgrid=unique([sort(matnetgrid(:,1:2),2),matnetgrid(:,3)],'rows') ;
                dlmwrite('WorkspaceRay/TempData/matnetgrid.txt',matnetgrid,'precision',textndigit,'-append','delimiter',' ');
                finmatnetgrid=dlmread('WorkspaceRay/TempData/matnetgrid.txt');
                finmatnetgrid=unique([sort(finmatnetgrid(:,1:2),2),finmatnetgrid(:,3)],'rows') ;
                dlmwrite('WorkspaceRay/TempData/finmatnetgrid.txt',finmatnetgrid,'precision',textndigit,'delimiter',' ');
                
            else
                finmatnetgrid=[];
                finmatnetgrid=dlmread('WorkspaceRay/TempData/finmatnetgrid.txt');
            end
            
            if  ReadMode==0
                
                uniqmatnetgrid=unique(round(finmatnetgrid(:,1:2)));
                dlmwrite('WorkspaceRay/TempData/uniqmatnetgrid.txt',uniqmatnetgrid,'precision',textndigit,'delimiter',' ');
                totG=graph(finmatnetgrid(:,1),finmatnetgrid(:,2),finmatnetgrid(:,3));
                maxnseglin=max(max(finmatnetgrid(:,1:2)));
                noncongrid=setxor(finmatnetgrid(:,1:2),1:maxnseglin);
                totGfilt=rmnode(totG,noncongrid);
                dlmwrite('WorkspaceRay/TempData/totGfilt.txt',[totGfilt.Edges.EndNodes,totGfilt.Edges.Weight],'delimiter',' ');
           
            else
                uniqmatnetgrid=[];
                uniqmatnetgrid=dlmread('WorkspaceRay/TempData/uniqmatnetgrid.txt');
                mat_totGfilt=[];
                mat_totGfilt=dlmread('WorkspaceRay/TempData/totGfilt.txt');
                totGfilt=graph(mat_totGfilt(:,1),mat_totGfilt(:,2),mat_totGfilt(:,3));
            end
            obj.final_network=finmatnetgrid;
            obj.uniq_line=uniqmatnetgrid;
            obj.final_graph=totGfilt;
        end
        


        function plotGraphWorkspace(obj)   %nsegvar= the division number of interval of variables
            
            nedges=length(obj.final_graph.Edges.Weight);
            figure;
            LWidths = 1./(obj.final_graph.Edges.Weight);
            plgraf=plot(obj.final_graph,'LineWidth',1*ones(nedges,1));  %,'NodeLabelMode','auto'  ,'EdgeLabel',round(LWidths,3)
            obj.final_graph.Edges.EdgesColors = (LWidths);
            plgraf.EdgeCData = obj.final_graph.Edges.EdgesColors;
            plgraf.NodeColor=[0.85,0.85,0.85];
            plgraf.MarkerSize = 0.1;
            colorbar
            set(gca,'xtick',[],'ytick',[])
          
        end
        
        
        function inv_TF=TF_Index(obj,magvar1,magvar2)
            
            ncable=obj.model.numCables;
            numDofs=obj.model.numDofs;
            
            obj.model.update(magvar1', zeros(numDofs,1), zeros(numDofs,1),zeros(numDofs,1));            
            linprogf=ones(ncable,1);
            linprogA=[];
            linprogb=[];
            linprogAeq=-(obj.model.L)';
            linprogbeq=zeros(numDofs,1);
            linproglb=0.1*ones(ncable,1);
            linprogub=[];
            options=optimset('Display', 'off');
            optT1 = linprog(linprogf,linprogA,linprogb,linprogAeq,linprogbeq,linproglb,linprogub,[],options);
            inv_TF1=max(optT1)/min(optT1);
            
            
            
            obj.model.update(magvar2', zeros(numDofs,1), zeros(numDofs,1),zeros(numDofs,1));
            linprogf=ones(ncable,1);
            linprogA=[];
            linprogb=[];
            linprogAeq=-(obj.model.L)';
            linprogbeq=zeros(numDofs,1);
            linproglb=0.1*ones(ncable,1);
            linprogub=[];
            options=optimset('Display', 'off');
            optT2 = linprog(linprogf,linprogA,linprogb,linprogAeq,linprogbeq,linproglb,linprogub,[],options);
            inv_TF2=max(optT2)/min(optT2);
            
            inv_TF=(inv_TF1+inv_TF2)/2;

            
        end
        
        
      
        
        
    end
end
    
    
    
    
    
    
    
