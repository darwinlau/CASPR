



classdef RayGraphGeneration < handle
    
    
    properties
        
        model
        grid
        workspace            % Grid object for brute force workspace (input)
        
        final_network
        uniq_line
        final_graph
    end
    
    
    methods
        
        function w = RayGraphGeneration(model,grid,workspace)
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
            
            [sortcol,nsortcol]=sort(matnod(:,1));
            [uniqnod,nuniqnod]=unique(sortcol);
            nuniqnod=[nuniqnod;length(sortcol)+1]; %number-unique-node
            
            
            if  ReadMode==0
                matnetlin=[];
                dlmwrite('WorkspaceRay/TempData/matnetlin.txt',matnetlin);
                lengnuniqnod=length(nuniqnod);
                for ituninod=1:lengnuniqnod-1
                    ncurconlin=matnod(nsortcol(nuniqnod(ituninod):nuniqnod(ituninod+1)-1),2);  %number-current-connected-lines
                    lengcurnet=length(ncurconlin);
                    if lengcurnet>1
                        segvar=obj.grid.nod2vect(uniqnod(ituninod));
                        magvar=obj.grid.getGridPoint(segvar);
                        if WeightCond==1
                            inv_TF=TF_Index(obj,magvar);
                        else
                            inv_TF=1;
                        end
                        curnet=[];
                        curnet=nchoosek(ncurconlin,2);
                        [rowcurnet colcurnet]=size(curnet);
                        curnet=[curnet,ones(rowcurnet,1)*inv_TF];
                        matnetlin=[matnetlin;curnet];
                    end
                    if mod(ituninod,itwritex)==0
                        ituninod
                        matnetlin=unique([sort(matnetlin(:,1:2),2),matnetlin(:,3)],'rows') ;
                        dlmwrite('WorkspaceRay/TempData/matnetlin.txt',matnetlin,'precision',textndigit,'-append','delimiter',' ');
                        matnetlin=[];
                    end
                end
                matnetlin=unique([sort(matnetlin(:,1:2),2),matnetlin(:,3)],'rows') ;
                dlmwrite('WorkspaceRay/TempData/matnetlin.txt',matnetlin,'precision',textndigit,'-append','delimiter',' ');
                finmatnetlin=dlmread('WorkspaceRay/TempData/matnetlin.txt');
                finmatnetlin=unique([sort(finmatnetlin(:,1:2),2),finmatnetlin(:,3)],'rows') ;
                dlmwrite('WorkspaceRay/TempData/finmatnetlin.txt',finmatnetlin,'precision',textndigit,'delimiter',' ');
            else
                finmatnetlin=[];
                finmatnetlin=dlmread('WorkspaceRay/TempData/finmatnetlin.txt');
            end
            
            if  ReadMode==0
                uniqmatnetlin=unique(finmatnetlin(:,1:2));
                dlmwrite('WorkspaceRay/TempData/uniqmatnetlin.txt',uniqmatnetlin,'delimiter',' ');
                totG=graph(finmatnetlin(:,1),finmatnetlin(:,2),finmatnetlin(:,3));
                maxnseglin=max(max(finmatnetlin(:,1:2)));
                nonconlin=setxor(finmatnetlin(:,1:2),1:maxnseglin);
                totGfilt=rmnode(totG,nonconlin);
                dlmwrite('WorkspaceRay/TempData/totGfilt.txt',[totGfilt.Edges.EndNodes,totGfilt.Edges.Weight],'precision',textndigit,'delimiter',' ');
            else
                uniqmatnetlin=[];
                uniqmatnetlin=dlmread('WorkspaceRay/TempData/uniqmatnetlin.txt');
                mat_totGfilt=[];
                mat_totGfilt=dlmread('WorkspaceRay/TempData/totGfilt.txt');
                totGfilt=graph(mat_totGfilt(:,1),mat_totGfilt(:,2),mat_totGfilt(:,3));
            end
            obj.final_network=finmatnetlin;
            obj.uniq_line=uniqmatnetlin;
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
        
        
        function inv_TF=TF_Index(obj,magvar)
            
            ncable=obj.model.numCables;
            numDofs=obj.model.numDofs;
            obj.model.update(magvar', zeros(numDofs,1), zeros(numDofs,1),zeros(numDofs,1));
 
            
            linprogf=ones(ncable,1);
            linprogA=[];
            linprogb=[];
            linprogAeq=-(obj.model.L)';
            linprogbeq=zeros(numDofs,1);
            linproglb=0.01*ones(ncable,1);
            linprogub=[];
            options=optimset('Display', 'off');
            optT = linprog(linprogf,linprogA,linprogb,linprogAeq,linprogbeq,linproglb,linprogub,[],options);
            inv_TF=max(optT)/min(optT);
            
        end
        
        
        
        
        
        function [shortpath,listgrid,matpathvar,TFvect]=Path_Generation(obj,start_pose,end_pose)
            
            
              startvect=obj.grid.var2vect(start_pose);
              startnod=obj.grid.vect2nod(startvect);
              nstartnod=find(obj.workspace.MatNodeGrid(:,1)==startnod);
              startseglin=obj.workspace.MatNodeGrid(nstartnod(1),2);
              [starttotGfilt,colling]=find(obj.uniq_line==startseglin);
 
              finalvect=obj.grid.var2vect(end_pose);
              finalnod=obj.grid.vect2nod(finalvect);
              nfinalnod=find(obj.workspace.MatNodeGrid(:,1)==finalnod);
              finalseglin=obj.workspace.MatNodeGrid(nfinalnod(1),2);
              [finaltotGfilt,colling]=find(obj.uniq_line==finalseglin);

              [shortpath,pathdist] = shortestpath(obj.final_graph,starttotGfilt,finaltotGfilt);
            
              
              
              nvar=obj.model.numDofs;
              gridi=startnod;
              listgrid=[gridi];
              linei=find(obj.workspace.MatRays(:,1)==obj.uniq_line(shortpath(1)));
              lineinumnzgrid=length(find(obj.workspace.MatRays(linei,nvar+4:end)>0));
              
              for itnpath=1:length(shortpath)-1
                  lineip=find(obj.workspace.MatRays(:,1)==obj.uniq_line(shortpath(itnpath+1)));
                  lineipnumnzgrid=length(find(obj.workspace.MatRays(lineip,nvar+4:end)>0));
                  
                  gridip=intersect(obj.workspace.MatRays(linei,nvar+4:nvar+4+lineinumnzgrid-1),obj.workspace.MatRays(lineip,nvar+4:nvar+4+lineipnumnzgrid-1));
                  
                  colgridi=nvar+3+find(obj.workspace.MatRays(linei,nvar+4:nvar+4+lineinumnzgrid-1)==gridi);
                  colgridip=nvar+3+find(obj.workspace.MatRays(linei,nvar+4:nvar+4+lineinumnzgrid-1)==gridip);
                  if colgridip>=colgridi
                      listgrid=[listgrid,obj.workspace.MatRays(linei,colgridi+1:colgridip)];
                      
                  else
                      listgrid=[listgrid,obj.workspace.MatRays(linei,colgridi-1:-1:colgridip)];
                  end
                  gridi=gridip;
                  linei=lineip;
                  lineinumnzgrid=lineipnumnzgrid;
              end
              gridip=finalnod;
              colgridi=nvar+3+find(obj.workspace.MatRays(linei,nvar+4:nvar+4+lineinumnzgrid-1)==gridi);
              colgridip=nvar+3+find(obj.workspace.MatRays(linei,nvar+4:nvar+4+lineinumnzgrid-1)==gridip);
              if colgridip>=colgridi
                  listgrid=[listgrid,obj.workspace.MatRays(linei,colgridi+1:colgridip)];
                  
              else
                  listgrid=[listgrid,obj.workspace.MatRays(linei,colgridi-1:-1:colgridip)];
              end             
              segvar=obj.grid.nod2vect(listgrid');

              [n_path_grid,spare]=size(segvar);
              
              for itgridpath=1:n_path_grid
                  matpathvar(itgridpath,:)=obj.grid.getGridPoint(segvar(itgridpath,:));
                  TFvect(itgridpath,1)=1/obj.TF_Index( matpathvar(itgridpath,:));
              end
            
        end
        
    
        
    end
end
    
    
    
    
    
    
    
