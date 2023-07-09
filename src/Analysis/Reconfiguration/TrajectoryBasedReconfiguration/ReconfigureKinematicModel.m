classdef ReconfigureKinematicModel < handle
    %UNTITLED8 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        L
        L_q
        Cable_Length
        Cable_Length_q
        Cable_Length_Poly
        L_Removed_Norm
        L_Removed_Norm_q
        Robot_Type    % 1 -> spatial; 2 -> plannar xz; 3-> plannar xy; 4-> ...
        Digit = 9;
        Reconfig_Element
        q_fun
        model
        ValidReconfigCableID
        ValidAuxLength
        
        
    end
    
    methods
        function obj = ReconfigureKinematicModel(model,Reconfig_Element,Robot_Type)
            obj.Reconfig_Element  =  Reconfig_Element;
            obj.model = model;
            for i = 1:model.numCables
                OA_0(:,i) =  model.cableModel.cables{i}.attachments{1}.r_GA ;
                OB_0(:,i) =  model.cableModel.cables{i}.attachments{2}.r_GA ;
            end
            OA_sym = sym(OA_0);
            OB_sym = sym(OB_0);
            
            q_sym = sym('q%d',size(model.q));
            obj.q_fun = q_sym;
            
            for i = 1:Reconfig_Element.Element_Count
                numVarRequired = numel(Reconfig_Element.Info(i).VariableID);
                var = Reconfig_Element.Info(i).Variable;
                OA_sym(:,Reconfig_Element.Reconfig_Cable_ID(i)) =  Reconfig_Element.Info(i).Path(var);
                clear var
            end
            OA_sym = vpa(OA_sym,obj.Digit);
            obj.Robot_Type = Robot_Type;
            if obj.Robot_Type == 1
                [L,L_Removed_Norm,Cable_Length] = obj.SpatialModel(OA_sym,OB_sym,q_sym);
            elseif obj.Robot_Type == 2
                [L,L_Removed_Norm,Cable_Length] = obj.PlanarXZ(OA_sym,OB_sym,q_sym);
            elseif obj.Robot_Type == 3
                [L,L_Removed_Norm,Cable_Length] = obj.PlanarXY(OA_sym,OB_sym,q_sym);
            else
                CASPR_log.Error('Please clarify your model in this class');
            end
            obj.L_q = vpa(L,obj.Digit);
            obj.L_Removed_Norm_q = vpa(L_Removed_Norm,obj.Digit);
            obj.Cable_Length_q = vpa(Cable_Length,obj.Digit);
        end
        
        function UpdateModel(obj,q_val,sym_cable_id,var_range)
            for i = 1:obj.model.numCables
                OA_0(:,i) =  obj.model.cableModel.cables{i}.attachments{1}.r_GA ;
                OB_0(:,i) =  obj.model.cableModel.cables{i}.attachments{2}.r_GA ;
            end
            obj.ValidReconfigCableID = sym_cable_id;
            
            obj.ValidAuxLength = obj.Reconfig_Element.Unknown_length(find(ismember(obj.Reconfig_Element.Reconfig_Cable_ID,obj.ValidReconfigCableID)));
            
            OA_sym = sym(OA_0);
            OB_sym = sym(OB_0);
            sym_cable_num = find(ismember(obj.Reconfig_Element.Reconfig_Cable_ID, sym_cable_id));
            
            
            for i = 1:numel(sym_cable_num)
                numVarRequired = numel(obj.Reconfig_Element.Info(sym_cable_num(i)).VariableID);
                var = obj.Reconfig_Element.Info(sym_cable_num(i)).Variable;
                OA_sym(:,sym_cable_id(i)) =  obj.Reconfig_Element.Info(sym_cable_num(i)).Path(var);
                clear var
            end
            
            OA_sym = vpa(OA_sym,obj.Digit);
            
            
            if obj.Robot_Type == 1
                [obj.L,obj.L_Removed_Norm,obj.Cable_Length] = obj.SpatialModel(OA_sym,OB_sym,q_val);
            elseif obj.Robot_Type == 2
                [obj.L,obj.L_Removed_Norm,obj.Cable_Length]  = obj.PlanarXZ(OA_sym,OB_sym,q_val);
            elseif obj.Robot_Type == 3
                [obj.L,obj.L_Removed_Norm,obj.Cable_Length]  = obj.PlanarXY(OA_sym,OB_sym,q_val);
            else
                CASPR_log.Error('Please clarify your model in this class');
            end
            
            
            
        end
        
        function ReplaceNormByPolynomials(obj,valid_CID,valid_u,constantVarID,valid_range,u_current,cable2varIdx)
            obj.Cable_Length_Poly = [];
%             for j = 1:numel(valid_u)
%                 t(j,:) = linspace(valid_range(1,j),valid_range(2,j),10);
%             end
%             
            for i = 1:numel(valid_CID)
                CurrentCable = valid_CID(i);
                
                IDX = find(CurrentCable == [obj.Reconfig_Element.Info.CableID]);
                
                NumVar = numel([obj.Reconfig_Element.Info(IDX).VariableID]);
                VarInd = obj.Reconfig_Element.Info(IDX).VariableID;
                if ~isempty(constantVarID)
                    inValidVarIDX = (VarInd==constantVarID);
                else
                    inValidVarIDX = [];
                end
%                 VarRange = valid_range(:,VarInd)*5;
                VarRange = valid_range(:,VarInd);
                Var = obj.Reconfig_Element.Info(IDX).Variable;  
                
                for j = 1:NumVar
                    Low =  VarRange(1,j);  Up =  VarRange(2,j);
                    samplePoint(j,:) = (Up-Low).*rand(50,1) + Low;
%                     samplePoint(j,:) = linspace(Low,Up,50);
                end
                
                for j = 1:size(samplePoint,2)
                    length(j) = double(subs(obj.Cable_Length(CurrentCable),Var,samplePoint(:,j)'));
                end
                
                samplePoint(inValidVarIDX,:) = [];
                Var(inValidVarIDX) = [];
                %                 if NumVar >=1
                p = polyfitn(samplePoint',length,3);
                
                
                sympoly = vpa(polyn2sym(p),7);
                
                u_tmp = Var;
                
                for j = 1:size(p.ModelTerms,2)                    
                    for k = 1:size(p.ModelTerms,1)
                       u_val(k,j) =  u_tmp(j)^ p.ModelTerms(k,j);
                    end                    
                end
                u_coeff = u_val(:,1);
                for j = 2:size(u_val,2)
                    u_coeff = u_coeff.*u_val(:,j);
                end
                
                obj.Cable_Length_Poly{i} = vpa(p.Coefficients* u_coeff,7);
                
                %                 elses
                %                     [poly_coeff,finess] = polyfit(samplePoint,length,3);
                %                 end
                %                 [xg,yg]=meshgrid(-0.05:0.005:0.05);
                %                 zg = polyvaln(p,[xg(:),yg(:)]);
                %                 surf(xg,yg,reshape(zg,size(xg)))
                %                 hold on
                %                 plot3(samplePoint(1,:),samplePoint(2,:),length,'o')
                %                 hold off
                
            end
            
            
            %             for i = 1:numel(valid_CID)
            %                 t = linspace(valid_range(1,i),valid_range(2,i),10);
            %                 cable_len_var = double(subs(obj.Cable_Length(valid_CID(i)),t));
            %                 var = valid_u(i);
            %                 [poly_coeff,finess] = polyfit(t,cable_len_var,3);
            %                 obj.Cable_Length_Poly{i} = poly_coeff*[var^3;var^2;var^1;var^0];
            %
            %             end
            
            
        end
        
    end
    
    methods(Static)
        function [final_L,final_L_bar,cable_length] = SpatialModel(OA,OB,q)
            %             a = OA;
            %             b = OB;
            %             c = size(OA,2);
            %             Rx = [1 0 0;0 cos(q(4)) -sin(q(4)); 0 sin(q(4)) cos(q(4))];
            %             Ry = [cos(q(5)) 0 sin(q(5));0 1 0;-sin(q(5))  0 cos(q(5))];
            %             Rz = [cos(q(6)) -sin(q(6)) 0; sin(q(6)) cos(q(6)) 0; 0 0 1];
            %             R = Rx*Ry*Rz;
            %             for i = 1:c
            %                 L_bar(1:3,i) = ([q(1);q(2);q(3)] + R*b(:,i) - a(:,i));
            %                 cable_length(:,i) =  sqrt(([q(1);q(2);q(3)] + R*b(:,i) - a(:,i)).' * ([q(1);q(2);q(3)] + R*b(:,i) - a(:,i)));
            %                 L(1:3,i)  =  ([q(1);q(2);q(3)] + R*b(:,i) - a(:,i)) / cable_length(:,i);
            %                 L_bar(4:6,i) = cross(-L_bar(1:3,i),R*b(:,i));
            %                 L(4:6,i) = cross(-L(1:3,i),R*b(:,i));
            %             end
            
            a = OA;
            b = OB;
            c = size(OA,2);
            A = q(4);
            B = q(5);
            G = q(6);
            
            R = [cos(B)*cos(G) -cos(B)*sin(G) sin(B);
                cos(A)*sin(G)+sin(A)*sin(B)*cos(G)  cos(A)*cos(G)-sin(A)*sin(B)*sin(G)  -sin(A)*cos(B);
                sin(A)*sin(G)-cos(A)*sin(B)*cos(G)  sin(A)*cos(G)+cos(A)*sin(B)*sin(G)  cos(A)*cos(B)];
            
            S = [cos(B)*cos(G) cos(A)*sin(G) + sin(A)*sin(B)*cos(G) sin(A)*sin(G) - cos(A)*sin(B)*cos(G) 0 0 0;
                -cos(B)*sin(G) cos(A)*cos(G) - sin(A)*sin(B)*sin(G) sin(A)*cos(G) + cos(A)*sin(B)*sin(G) 0 0 0;
                sin(B)           -sin(A)*cos(B)                      cos(A)*cos(B)                      0 0 0;
                0                    0                                    0                             cos(B)*cos(G) sin(G) 0;
                0                    0                                    0                            -cos(B)*sin(G) cos(G) 0;
                0                    0                                    0                             sin(B)          0    1];
            
            
            for i = 1:c
                cable_length(:,i) =  sqrt(((-[q(1);q(2);q(3)] - R*b(:,i) + a(:,i))).' * ((-[q(1);q(2);q(3)] - R*b(:,i) + a(:,i))));
                L_bar(1:3,i) = -R'*(-[q(1);q(2);q(3)] - R*b(:,i) + a(:,i));
                L(1:3,i)  =  -R'*(-[q(1);q(2);q(3)] - R*b(:,i) + a(:,i)) / cable_length(:,i);
                L_bar(4:6,i) = cross(-L_bar(1:3,i),b(:,i));
                L(4:6,i) = cross(-L(1:3,i),b(:,i));
            end
            final_L = -S'*L;
            final_L_bar = -S'*L_bar;
            
        end
        
        function [L,L_bar,cable_length] = PlanarXZ(OA,OB,q)
            
            a = OA;
            b = OB;
            c = size(OA,2);
            R = [cos(q(3)) 0 sin(q(3));0 1 0; -sin(q(3)) 0 cos(q(3))];
            for i = 1:size(OA,2)
                L_bar(1:3,i) = ([q(1);0;q(2)] + R*b(:,i) - a(:,i)); %/ norm([q(1);0;q(2)] + R*b(:,i) - a(:,i))
                cable_length(:,i) = sqrt(([q(1);0;q(2)] + R*b(:,i) - a(:,i)).'*([q(1);0;q(2)] + R*b(:,i) - a(:,i)));
                L(1:3,i) = ([q(1);0;q(2)] + R*b(:,i) - a(:,i))/(cable_length(:,i));
                L_bar(4:6,i) = cross(-L_bar(1:3,i),R*b(:,i));
                L(4:6,i) = cross(-L(1:3,i),R*b(:,i));
            end
            L_bar([2,4,6],:) = [];
            L([2,4,6],:) = [];
        end
        
        function [L,L_bar,cable_length] = PlanarXY(OA,OB,q)
            a = OA;
            b = OB;
            c = size(OA,2);
            R = [cos(q(3)) -sin(q(3)) 0; sin(q(3)) cos(q(3)) 0; 0 0 1];
            %             R = [cos(q(3)) 0 sin(q(3));0 1 0; -sin(q(3)) 0 cos(q(3))];
            for i = 1:size(OA,2)
                L_bar(1:3,i) = ([q(1);q(2);0] + R*b(:,i) - a(:,i)); %/ norm([q(1);0;q(2)] + R*b(:,i) - a(:,i))
                cable_length(:,i) = sqrt(([q(1);q(2);0] + R*b(:,i) - a(:,i)).'*([q(1);q(2);0] + R*b(:,i) - a(:,i)));
                L(1:3,i) = ([q(1);q(2);0] + R*b(:,i) - a(:,i))/(cable_length(:,i));
                L_bar(4:6,i) = cross(-L_bar(1:3,i),R*b(:,i));
                L(4:6,i) = cross(-L(1:3,i),R*b(:,i));
            end
            L_bar([3,4,5],:) = [];
            L([3,4,5],:) = [];
        end
    end
end