classdef HyperplaneWorkspaceWrenchClosure < WorkspaceCondition
    %IDFUNCTION Summary of this class goes here
    %   Detailed explanation goes here
    
    properties (SetAccess = protected, GetAccess = protected)
        semi_singular_type              % The method for determining semisingular values
    end
    
    methods
        %% Constructor for wrench closure workspace
        function w = HyperplaneWorkspaceWrenchClosure(semi_singular_type)
            w.semi_singular_type    = semi_singular_type;
        end
        
        %% Evaluate the wrench closure condition return true if satisfied 
        function [inWorkspace,semi_singular] = evaluate(obj,dynamics)
            % -------------------------------------------------------------
            % This function tests if the cable joint jacobian positively
            % spans the space.
            % -------------------------------------------------------------
            if(obj.semi_singular_type == 0)
                % Determine necessary variables for test             
                L       =   dynamics.L; % Cable Jacobian
                L_rank  =   rank(L');   % Cable Jacobian Rank
                if(L_rank ~= dynamics.numDofs)
                   inWorkspace = 0;
                   semi_singular = 0;
                else
                   numPlanes           =   nchoosek(dynamics.numCables,dynamics.numDofs - 1);
                   Gram                =   zeros(numPlanes,dynamics.numCables); % This matrix could also be used for separating hyperplane test
                   Varsigma            =   zeros(numPlanes,1);
                   if(dynamics.numDofs == 2)
                   elseif(dynamics.numDofs == 3)
                       hyperplane_index = zeros(dynamics.numDofs-1,numPlanes);
                       k = 1;
                       for i = 1:dynamics.numCables
                           for j = i+1:dynamics.numCables
                               hyperplane_index(:,k) = [i;j];
                               k = k+1;
                           end
                       end
                   end
                   i = 1;
                   pass = 1;
                   while((pass)&&(i<=numPlanes))
                       % Compute the Gram matrix
                       if(dynamics.numDofs == 2)
                           perp = [L(i,2);-L(i,1)];
                           no_test_list = i;
                       elseif(dynamics.numDofs == 3)
                           perp = cross(L(hyperplane_index(1,i),:),L(hyperplane_index(2,i),:))';
                           no_test_list = hyperplane_index(:,i);
                       end
                       for j = 1:dynamics.numCables
                           if(sum(j == no_test_list)>0)
                           else
                               Gram(i,j) = L(j,:)*perp/(norm(perp));
                           end
                       end
                       % Determine the necessary angle for each side of the
                       % hyperplane
                       if((sum(Gram(i,:)>0)>1e-10)&&(sum(Gram(i,:)<-1e-10)>0))
                           temp_pos = max(Gram(i,:));
                           temp_neg = min(Gram(i,:));
                           Varsigma(i) = sqrt(min(temp_neg^2,temp_pos^2));
                       else
                           pass = 0;
                       end
                       % Update semi_singular value if smaller than previous
                       i = i + 1;
                   end
                   if(pass == 1)
    %                    Varsigma
    %                    L
    %                    Gram
                       Varsigma2 = ones(3,1);
                       for i =1:3
                           for j = 1:3
                               if(i==j)

                               else
                                   Varsigma2(i) = abs(Gram(j,i))*Varsigma2(i);
                               end
                           end
                       end
                       nd = max(diag(L*L'));
    %                    Varsigma2
                       inWorkspace = 1;
                       semi_singular = 50*min(Varsigma)+1;
    %                     semi_singular = 1;
                       disp('Yes')
                   else
                       inWorkspace = 0;
                       semi_singular = 0;
                       disp('no no no')
                   end 
                end
            elseif(obj.semi_singular_type == 1)
                %% AT THIS POINT THE WORKSPACE GENERATION IS MODEL SPECIFIC (I HAVE NOT GOT THE CODE CALCULATING THE SUBJACOBIAN)
                %  THIS IS JUST the 4 cable 2R model
                % Determine necessary variables for test             
                L       =   dynamics.L; % Cable Jacobian
                L_rank  =   rank(L');   % Cable Jacobian Rank
                %% THIS IS TO BE REMOVED LATER
                LL1 = [(18*10^(1/2)*cos(dynamics.q(1)) - 129*10^(1/2) + 144*10^(1/2)*sin(dynamics.q(1)) + 63*10^(1/2)*cos(dynamics.q(1))^2 - 16*10^(1/2)*cos(dynamics.q(1))*sin(dynamics.q(1)))/(40*(9 - 8*sin(dynamics.q(1)) - cos(dynamics.q(1)))^(3/2)),0;0,0];
                LL2 = [ (63*5^(1/2)*cos(2*dynamics.q(1) + 2*dynamics.q(2)) - 16*5^(1/2)*sin(2*dynamics.q(1) + 2*dynamics.q(2)) - 595*5^(1/2) + 73*5^(1/2)*cos(dynamics.q(1) + dynamics.q(2)) + 584*5^(1/2)*sin(dynamics.q(1) + dynamics.q(2)) - 480*5^(1/2)*cos(dynamics.q(2)) + 660*5^(1/2)*sin(dynamics.q(1)) + 60*5^(1/2)*sin(dynamics.q(2)) - 20*5^(1/2)*cos(dynamics.q(1) - dynamics.q(2)) + 32*5^(1/2)*cos(dynamics.q(1) + 2*dynamics.q(2)) + 160*5^(1/2)*cos(2*dynamics.q(1) + dynamics.q(2)) + 160*5^(1/2)*sin(dynamics.q(1) - dynamics.q(2)) + 126*5^(1/2)*sin(dynamics.q(1) + 2*dynamics.q(2)) - 20*5^(1/2)*sin(2*dynamics.q(1) + dynamics.q(2)) + 200*5^(1/2)*cos(dynamics.q(1))^2)/(5*(32*cos(dynamics.q(2)) - 32*sin(dynamics.q(1) + dynamics.q(2)) - 4*cos(dynamics.q(1) + dynamics.q(2)) - 40*sin(dynamics.q(1)) - 4*sin(dynamics.q(2)) + 53)^(3/2)),                                                         (63*5^(1/2)*cos(2*dynamics.q(1) + 2*dynamics.q(2)) - 16*5^(1/2)*sin(2*dynamics.q(1) + 2*dynamics.q(2)) - 195*5^(1/2) + 43*5^(1/2)*cos(dynamics.q(1) + dynamics.q(2)) + 344*5^(1/2)*sin(dynamics.q(1) + dynamics.q(2)) - 240*5^(1/2)*cos(dynamics.q(2)) + 195*5^(1/2)*sin(dynamics.q(1)) + 30*5^(1/2)*sin(dynamics.q(2)) - 10*5^(1/2)*cos(dynamics.q(1) - dynamics.q(2)) + 16*5^(1/2)*cos(dynamics.q(1) + 2*dynamics.q(2)) + 80*5^(1/2)*cos(2*dynamics.q(1) + dynamics.q(2)) + 80*5^(1/2)*sin(dynamics.q(1) - dynamics.q(2)) + 63*5^(1/2)*sin(dynamics.q(1) + 2*dynamics.q(2)) - 10*5^(1/2)*sin(2*dynamics.q(1) + dynamics.q(2)))/(5*(32*cos(dynamics.q(2)) - 32*sin(dynamics.q(1) + dynamics.q(2)) - 4*cos(dynamics.q(1) + dynamics.q(2)) - 40*sin(dynamics.q(1)) - 4*sin(dynamics.q(2)) + 53)^(3/2));(63*5^(1/2)*cos(2*dynamics.q(1) + 2*dynamics.q(2)) - 16*5^(1/2)*sin(2*dynamics.q(1) + 2*dynamics.q(2)) - 195*5^(1/2) + 43*5^(1/2)*cos(dynamics.q(1) + dynamics.q(2)) + 344*5^(1/2)*sin(dynamics.q(1) + dynamics.q(2)) - 240*5^(1/2)*cos(dynamics.q(2)) + 195*5^(1/2)*sin(dynamics.q(1)) + 30*5^(1/2)*sin(dynamics.q(2)) - 10*5^(1/2)*cos(dynamics.q(1) - dynamics.q(2)) + 16*5^(1/2)*cos(dynamics.q(1) + 2*dynamics.q(2)) + 80*5^(1/2)*cos(2*dynamics.q(1) + dynamics.q(2)) + 80*5^(1/2)*sin(dynamics.q(1) - dynamics.q(2)) + 63*5^(1/2)*sin(dynamics.q(1) + 2*dynamics.q(2)) - 10*5^(1/2)*sin(2*dynamics.q(1) + dynamics.q(2)))/(5*(32*cos(dynamics.q(2)) - 32*sin(dynamics.q(1) + dynamics.q(2)) - 4*cos(dynamics.q(1) + dynamics.q(2)) - 40*sin(dynamics.q(1)) - 4*sin(dynamics.q(2)) + 53)^(3/2)), (63*5^(1/2)*cos(2*dynamics.q(1) + 2*dynamics.q(2)) - 16*5^(1/2)*sin(2*dynamics.q(1) + 2*dynamics.q(2)) - 327*5^(1/2) + 73*5^(1/2)*cos(dynamics.q(1) + dynamics.q(2)) + 584*5^(1/2)*sin(dynamics.q(1) + dynamics.q(2)) - 584*5^(1/2)*cos(dynamics.q(2)) + 390*5^(1/2)*sin(dynamics.q(1)) + 73*5^(1/2)*sin(dynamics.q(2)) - 20*5^(1/2)*cos(dynamics.q(1) - dynamics.q(2)) + 32*5^(1/2)*cos(dynamics.q(1) + 2*dynamics.q(2)) + 160*5^(1/2)*cos(2*dynamics.q(1) + dynamics.q(2)) + 160*5^(1/2)*sin(dynamics.q(1) - dynamics.q(2)) + 126*5^(1/2)*sin(dynamics.q(1) + 2*dynamics.q(2)) - 20*5^(1/2)*sin(2*dynamics.q(1) + dynamics.q(2)) - 126*5^(1/2)*cos(dynamics.q(2))^2 + 32*5^(1/2)*cos(dynamics.q(2))*sin(dynamics.q(2)))/(5*(32*cos(dynamics.q(2)) - 32*sin(dynamics.q(1) + dynamics.q(2)) - 4*cos(dynamics.q(1) + dynamics.q(2)) - 40*sin(dynamics.q(1)) - 4*sin(dynamics.q(2)) + 53)^(3/2))];
                LL3 = [ (18*10^(1/2)*cos(dynamics.q(1)) - 129*10^(1/2) - 144*10^(1/2)*sin(dynamics.q(1)) + 63*10^(1/2)*cos(dynamics.q(1))^2 + 16*10^(1/2)*cos(dynamics.q(1))*sin(dynamics.q(1)))/(40*(8*sin(dynamics.q(1)) - cos(dynamics.q(1)) + 9)^(3/2)),0;0,0];
                LL4 = [ -(595*5^(1/2) - 16*5^(1/2)*sin(2*dynamics.q(1) + 2*dynamics.q(2)) - 63*5^(1/2)*cos(2*dynamics.q(1) + 2*dynamics.q(2)) - 73*5^(1/2)*cos(dynamics.q(1) + dynamics.q(2)) + 584*5^(1/2)*sin(dynamics.q(1) + dynamics.q(2)) + 480*5^(1/2)*cos(dynamics.q(2)) + 660*5^(1/2)*sin(dynamics.q(1)) + 60*5^(1/2)*sin(dynamics.q(2)) + 20*5^(1/2)*cos(dynamics.q(1) - dynamics.q(2)) - 32*5^(1/2)*cos(dynamics.q(1) + 2*dynamics.q(2)) - 160*5^(1/2)*cos(2*dynamics.q(1) + dynamics.q(2)) + 160*5^(1/2)*sin(dynamics.q(1) - dynamics.q(2)) + 126*5^(1/2)*sin(dynamics.q(1) + 2*dynamics.q(2)) - 20*5^(1/2)*sin(2*dynamics.q(1) + dynamics.q(2)) - 200*5^(1/2)*cos(dynamics.q(1))^2)/(5*(32*sin(dynamics.q(1) + dynamics.q(2)) - 4*cos(dynamics.q(1) + dynamics.q(2)) + 32*cos(dynamics.q(2)) + 40*sin(dynamics.q(1)) + 4*sin(dynamics.q(2)) + 53)^(3/2)),                                                         -(195*5^(1/2) - 16*5^(1/2)*sin(2*dynamics.q(1) + 2*dynamics.q(2)) - 63*5^(1/2)*cos(2*dynamics.q(1) + 2*dynamics.q(2)) - 43*5^(1/2)*cos(dynamics.q(1) + dynamics.q(2)) + 344*5^(1/2)*sin(dynamics.q(1) + dynamics.q(2)) + 240*5^(1/2)*cos(dynamics.q(2)) + 195*5^(1/2)*sin(dynamics.q(1)) + 30*5^(1/2)*sin(dynamics.q(2)) + 10*5^(1/2)*cos(dynamics.q(1) - dynamics.q(2)) - 16*5^(1/2)*cos(dynamics.q(1) + 2*dynamics.q(2)) - 80*5^(1/2)*cos(2*dynamics.q(1) + dynamics.q(2)) + 80*5^(1/2)*sin(dynamics.q(1) - dynamics.q(2)) + 63*5^(1/2)*sin(dynamics.q(1) + 2*dynamics.q(2)) - 10*5^(1/2)*sin(2*dynamics.q(1) + dynamics.q(2)))/(5*(32*sin(dynamics.q(1) + dynamics.q(2)) - 4*cos(dynamics.q(1) + dynamics.q(2)) + 32*cos(dynamics.q(2)) + 40*sin(dynamics.q(1)) + 4*sin(dynamics.q(2)) + 53)^(3/2));-(195*5^(1/2) - 16*5^(1/2)*sin(2*dynamics.q(1) + 2*dynamics.q(2)) - 63*5^(1/2)*cos(2*dynamics.q(1) + 2*dynamics.q(2)) - 43*5^(1/2)*cos(dynamics.q(1) + dynamics.q(2)) + 344*5^(1/2)*sin(dynamics.q(1) + dynamics.q(2)) + 240*5^(1/2)*cos(dynamics.q(2)) + 195*5^(1/2)*sin(dynamics.q(1)) + 30*5^(1/2)*sin(dynamics.q(2)) + 10*5^(1/2)*cos(dynamics.q(1) - dynamics.q(2)) - 16*5^(1/2)*cos(dynamics.q(1) + 2*dynamics.q(2)) - 80*5^(1/2)*cos(2*dynamics.q(1) + dynamics.q(2)) + 80*5^(1/2)*sin(dynamics.q(1) - dynamics.q(2)) + 63*5^(1/2)*sin(dynamics.q(1) + 2*dynamics.q(2)) - 10*5^(1/2)*sin(2*dynamics.q(1) + dynamics.q(2)))/(5*(32*sin(dynamics.q(1) + dynamics.q(2)) - 4*cos(dynamics.q(1) + dynamics.q(2)) + 32*cos(dynamics.q(2)) + 40*sin(dynamics.q(1)) + 4*sin(dynamics.q(2)) + 53)^(3/2)), -(327*5^(1/2) - 16*5^(1/2)*sin(2*dynamics.q(1) + 2*dynamics.q(2)) - 63*5^(1/2)*cos(2*dynamics.q(1) + 2*dynamics.q(2)) - 73*5^(1/2)*cos(dynamics.q(1) + dynamics.q(2)) + 584*5^(1/2)*sin(dynamics.q(1) + dynamics.q(2)) + 584*5^(1/2)*cos(dynamics.q(2)) + 390*5^(1/2)*sin(dynamics.q(1)) + 73*5^(1/2)*sin(dynamics.q(2)) + 20*5^(1/2)*cos(dynamics.q(1) - dynamics.q(2)) - 32*5^(1/2)*cos(dynamics.q(1) + 2*dynamics.q(2)) - 160*5^(1/2)*cos(2*dynamics.q(1) + dynamics.q(2)) + 160*5^(1/2)*sin(dynamics.q(1) - dynamics.q(2)) + 126*5^(1/2)*sin(dynamics.q(1) + 2*dynamics.q(2)) - 20*5^(1/2)*sin(2*dynamics.q(1) + dynamics.q(2)) + 126*5^(1/2)*cos(dynamics.q(2))^2 + 32*5^(1/2)*cos(dynamics.q(2))*sin(dynamics.q(2)))/(5*(32*sin(dynamics.q(1) + dynamics.q(2)) - 4*cos(dynamics.q(1) + dynamics.q(2)) + 32*cos(dynamics.q(2)) + 40*sin(dynamics.q(1)) + 4*sin(dynamics.q(2)) + 53)^(3/2))];
                LL2I = inv(LL2')*inv(LL2);
                LL4I = inv(LL4')*inv(LL4);
                %% END REMOVAL SECTION
                if(L_rank ~= dynamics.numDofs)
                   inWorkspace = 0;
                   semi_singular = 0;
                else
                   numPlanes           =   nchoosek(dynamics.numCables,dynamics.numDofs - 1);
                   Gram                =   zeros(numPlanes,dynamics.numCables); % This matrix could also be used for separating hyperplane test
                   Dist                =   zeros(numPlanes,dynamics.numCables);
                   Varsigma            =   zeros(numPlanes,1);
                   if(dynamics.numDofs == 2)
                   elseif(dynamics.numDofs == 3)
                       hyperplane_index = zeros(dynamics.numDofs-1,numPlanes);
                       k = 1;
                       for i = 1:dynamics.numCables
                           for j = i+1:dynamics.numCables
                               hyperplane_index(:,k) = [i;j];
                               k = k+1;
                           end
                       end
                   end
                   i = 1;
                   pass = 1;
                   while((pass)&&(i<=numPlanes))
                       % Compute the Gram matrix
                       if(dynamics.numDofs == 2)
                           perp = [L(i,2);-L(i,1)];
                           no_test_list = i;
                       elseif(dynamics.numDofs == 3)
                           perp = cross(L(hyperplane_index(1,i),:),L(hyperplane_index(2,i),:))';
                           no_test_list = hyperplane_index(:,i);
                       end
                       for j = 1:dynamics.numCables
                           if(sum(j == no_test_list)>0)
                           else
                               Gram(i,j) = L(j,:)*perp/(norm(perp));
                               if((j == 1) || (j==3))
                                   if(Gram(i,j) == 0)
                                       Dist(i,j) = 0;
                                   else
                                       if(j == 1)
                                           Dist(i,j) = sign(Gram(i,j))*abs(-L(j,1)/LL1(1,1));
                                       else
                                           Dist(i,j) = sign(Gram(i,j))*abs(-L(j,1)/LL3(1,1));
                                       end
                                   end
                               else
                                   if(j == 2)
                                       lamb = (L(i,1)*(LL2I(1,1)*L(j,1) + LL2I(1,2)*L(j,2)) + L(i,2)*(LL2I(2,1)*L(j,1) + LL2I(2,2)*L(j,2)))/(LL2I(1,1)*L(i,1)^2 + L(i,1)*L(i,2)*(LL2I(1,2) + LL2I(2,1)) + LL2I(2,2)*L(i,2)^2);
                                       tn = lamb*L(i,:);
                                       Dist(i,j) = sign(Gram(i,j))*norm(inv(LL2)*((L(j,:) - tn)'));
                                   elseif(j == 4)
                                       lamb = (L(i,1)*(LL4I(1,1)*L(j,1) + LL4I(1,2)*L(j,2)) + L(i,2)*(LL4I(2,1)*L(j,1) + LL4I(2,2)*L(j,2)))/(LL4I(1,1)*L(i,1)^2 + L(i,1)*L(i,2)*(LL4I(1,2) + LL4I(2,1)) + LL4I(2,2)*L(i,2)^2);
                                       tn = lamb*L(i,:);
                                       Dist(i,j) = sign(Gram(i,j))*norm(inv(LL4)*((L(j,:) - tn)'));
                                   end
                                   
                               end
                           end
                       end
                       % Determine the necessary angle for each side of the
                       % hyperplane
                       if((sum(Gram(i,:)>0)>1e-10)&&(sum(Gram(i,:)<-1e-10)>0))
                           temp_pos = max(Dist(i,:));
                           temp_neg = min(Dist(i,:));
                           Varsigma(i) = 0.2*sqrt(min(temp_neg^2,temp_pos^2));
                       else
                           pass = 0;
                       end
                       % Update semi_singular value if smaller than previous
                       i = i + 1;
                   end
                   if(pass == 1)
    %                    Varsigma
    %                    L
    %                    Gram
                       Varsigma2 = ones(3,1);
                       for i =1:3
                           for j = 1:3
                               if(i==j)

                               else
                                   Varsigma2(i) = abs(Gram(j,i))*Varsigma2(i);
                               end
                           end
                       end
                       nd = max(diag(L*L'));
    %                    Varsigma2
                       inWorkspace = 1;
                       semi_singular = 50*min(Varsigma)+2;
    %                     semi_singular = 1;
                       disp('Yes')
                   else
                       inWorkspace = 0;
                       semi_singular = 0;
                       disp('no no no')
                   end 
                end
            elseif(obj.semi_singular_type == 2)
                %% AT THIS POINT THE WORKSPACE GENERATION IS MODEL SPECIFIC (I HAVE NOT GOT THE CODE CALCULATING THE SUBJACOBIAN)
                %  THIS IS JUST the 3 cable 2R model
                % Determine necessary variables for test
                L       =   dynamics.L; % Cable Jacobian
                L_rank  =   rank(L');   % Cable Jacobian Rank
                %% THIS IS TO BE REMOVED LATER
                LL1 = [(18*10^(1/2)*cos(dynamics.q(1)) - 129*10^(1/2) + 144*10^(1/2)*sin(dynamics.q(1)) + 63*10^(1/2)*cos(dynamics.q(1))^2 - 16*10^(1/2)*cos(dynamics.q(1))*sin(dynamics.q(1)))/(40*(9 - 8*sin(dynamics.q(1)) - cos(dynamics.q(1)))^(3/2)),0;0,0];
                LL2 = [(63*5^(1/2)*cos(2*dynamics.q(1) + 2*dynamics.q(2)) - 16*5^(1/2)*sin(2*dynamics.q(1) + 2*dynamics.q(2)) - 595*5^(1/2) + 73*5^(1/2)*cos(dynamics.q(1) + dynamics.q(2)) + 584*5^(1/2)*sin(dynamics.q(1) + dynamics.q(2)) - 480*5^(1/2)*cos(dynamics.q(2)) + 660*5^(1/2)*sin(dynamics.q(1)) + 60*5^(1/2)*sin(dynamics.q(2)) - 20*5^(1/2)*cos(dynamics.q(1) - dynamics.q(2)) + 32*5^(1/2)*cos(dynamics.q(1) + 2*dynamics.q(2)) + 160*5^(1/2)*cos(2*dynamics.q(1) + dynamics.q(2)) + 160*5^(1/2)*sin(dynamics.q(1) - dynamics.q(2)) + 126*5^(1/2)*sin(dynamics.q(1) + 2*dynamics.q(2)) - 20*5^(1/2)*sin(2*dynamics.q(1) + dynamics.q(2)) + 200*5^(1/2)*cos(dynamics.q(1))^2)/(5*(32*cos(dynamics.q(2)) - 32*sin(dynamics.q(1) + dynamics.q(2)) - 4*cos(dynamics.q(1) + dynamics.q(2)) - 40*sin(dynamics.q(1)) - 4*sin(dynamics.q(2)) + 53)^(3/2)),(63*5^(1/2)*cos(2*dynamics.q(1) + 2*dynamics.q(2)) - 16*5^(1/2)*sin(2*dynamics.q(1) + 2*dynamics.q(2)) - 195*5^(1/2) + 43*5^(1/2)*cos(dynamics.q(1) + dynamics.q(2)) + 344*5^(1/2)*sin(dynamics.q(1) + dynamics.q(2)) - 240*5^(1/2)*cos(dynamics.q(2)) + 195*5^(1/2)*sin(dynamics.q(1)) + 30*5^(1/2)*sin(dynamics.q(2)) - 10*5^(1/2)*cos(dynamics.q(1) - dynamics.q(2)) + 16*5^(1/2)*cos(dynamics.q(1) + 2*dynamics.q(2)) + 80*5^(1/2)*cos(2*dynamics.q(1) + dynamics.q(2)) + 80*5^(1/2)*sin(dynamics.q(1) - dynamics.q(2)) + 63*5^(1/2)*sin(dynamics.q(1) + 2*dynamics.q(2)) - 10*5^(1/2)*sin(2*dynamics.q(1) + dynamics.q(2)))/(5*(32*cos(dynamics.q(2)) - 32*sin(dynamics.q(1) + dynamics.q(2)) - 4*cos(dynamics.q(1) + dynamics.q(2)) - 40*sin(dynamics.q(1)) - 4*sin(dynamics.q(2)) + 53)^(3/2));(63*5^(1/2)*cos(2*dynamics.q(1) + 2*dynamics.q(2)) - 16*5^(1/2)*sin(2*dynamics.q(1) + 2*dynamics.q(2)) - 195*5^(1/2) + 43*5^(1/2)*cos(dynamics.q(1) + dynamics.q(2)) + 344*5^(1/2)*sin(dynamics.q(1) + dynamics.q(2)) - 240*5^(1/2)*cos(dynamics.q(2)) + 195*5^(1/2)*sin(dynamics.q(1)) + 30*5^(1/2)*sin(dynamics.q(2)) - 10*5^(1/2)*cos(dynamics.q(1) - dynamics.q(2)) + 16*5^(1/2)*cos(dynamics.q(1) + 2*dynamics.q(2)) + 80*5^(1/2)*cos(2*dynamics.q(1) + dynamics.q(2)) + 80*5^(1/2)*sin(dynamics.q(1) - dynamics.q(2)) + 63*5^(1/2)*sin(dynamics.q(1) + 2*dynamics.q(2)) - 10*5^(1/2)*sin(2*dynamics.q(1) + dynamics.q(2)))/(5*(32*cos(dynamics.q(2)) - 32*sin(dynamics.q(1) + dynamics.q(2)) - 4*cos(dynamics.q(1) + dynamics.q(2)) - 40*sin(dynamics.q(1)) - 4*sin(dynamics.q(2)) + 53)^(3/2)), (63*5^(1/2)*cos(2*dynamics.q(1) + 2*dynamics.q(2)) - 16*5^(1/2)*sin(2*dynamics.q(1) + 2*dynamics.q(2)) - 327*5^(1/2) + 73*5^(1/2)*cos(dynamics.q(1) + dynamics.q(2)) + 584*5^(1/2)*sin(dynamics.q(1) + dynamics.q(2)) - 584*5^(1/2)*cos(dynamics.q(2)) + 390*5^(1/2)*sin(dynamics.q(1)) + 73*5^(1/2)*sin(dynamics.q(2)) - 20*5^(1/2)*cos(dynamics.q(1) - dynamics.q(2)) + 32*5^(1/2)*cos(dynamics.q(1) + 2*dynamics.q(2)) + 160*5^(1/2)*cos(2*dynamics.q(1) + dynamics.q(2)) + 160*5^(1/2)*sin(dynamics.q(1) - dynamics.q(2)) + 126*5^(1/2)*sin(dynamics.q(1) + 2*dynamics.q(2)) - 20*5^(1/2)*sin(2*dynamics.q(1) + dynamics.q(2)) - 126*5^(1/2)*cos(dynamics.q(2))^2 + 32*5^(1/2)*cos(dynamics.q(2))*sin(dynamics.q(2)))/(5*(32*cos(dynamics.q(2)) - 32*sin(dynamics.q(1) + dynamics.q(2)) - 4*cos(dynamics.q(1) + dynamics.q(2)) - 40*sin(dynamics.q(1)) - 4*sin(dynamics.q(2)) + 53)^(3/2))];
                LL3 = [ (18*10^(1/2)*cos(dynamics.q(1)) - 129*10^(1/2) - 144*10^(1/2)*sin(dynamics.q(1)) + 63*10^(1/2)*cos(dynamics.q(1))^2 + 16*10^(1/2)*cos(dynamics.q(1))*sin(dynamics.q(1)))/(40*(8*sin(dynamics.q(1)) - cos(dynamics.q(1)) + 9)^(3/2)),0;0,-(8587*cos(dynamics.q(2)) + 4986*sin(dynamics.q(2)) + 2232*cos(dynamics.q(2))*sin(dynamics.q(2)) + 1274*cos(dynamics.q(2))^2 + 3218)/(10*(124*cos(dynamics.q(2)) + 72*sin(dynamics.q(2)) + 277)^(3/2))];
                LL2I = inv(LL2')*inv(LL2);
                LL3I = inv(LL3')*inv(LL3);
                %% END REMOVAL SECTION
                if(L_rank ~= dynamics.numDofs)
                    inWorkspace = 0;
                    semi_singular = 0;
                else
                    numPlanes           =   nchoosek(dynamics.numCables,dynamics.numDofs - 1);
                    Gram                =   zeros(numPlanes,dynamics.numCables); % This matrix could also be used for separating hyperplane test
                    Dist                =   zeros(numPlanes,dynamics.numCables);
                    Varsigma            =   zeros(numPlanes,1);
                    if(dynamics.numDofs == 2)
                    elseif(dynamics.numDofs == 3)
                        hyperplane_index = zeros(dynamics.numDofs-1,numPlanes);
                        k = 1;
                        for i = 1:dynamics.numCables
                            for j = i+1:dynamics.numCables
                                hyperplane_index(:,k) = [i;j];
                                k = k+1;
                            end
                        end
                    end
                    i = 1;
                    pass = 1;
                    while((pass)&&(i<=numPlanes))
                        % Compute the Gram matrix
                        if(dynamics.numDofs == 2)
                            perp = [L(i,2);-L(i,1)];
                            no_test_list = i;
                        elseif(dynamics.numDofs == 3)
                            perp = cross(L(hyperplane_index(1,i),:),L(hyperplane_index(2,i),:))';
                            no_test_list = hyperplane_index(:,i);
                        end
                        for j = 1:dynamics.numCables
                            if(sum(j == no_test_list)>0)
                            else
                                Gram(i,j) = L(j,:)*perp/(norm(perp));
                                if(j == 1)
                                    if(Gram(i,j) == 0)
                                        Dist(i,j) = 0;
                                    else
                                        Dist(i,j) = sign(Gram(i,j))*abs(-L(j,1)/LL1(1,1));
                                    end
                                else
                                    if(j == 2)
                                        lamb = (L(i,1)*(LL2I(1,1)*L(j,1) + LL2I(1,2)*L(j,2)) + L(i,2)*(LL2I(2,1)*L(j,1) + LL2I(2,2)*L(j,2)))/(LL2I(1,1)*L(i,1)^2 + L(i,1)*L(i,2)*(LL2I(1,2) + LL2I(2,1)) + LL2I(2,2)*L(i,2)^2);
                                        tn = lamb*L(i,:);
                                        Dist(i,j) = sign(Gram(i,j))*norm(inv(LL2)*((L(j,:) - tn)'));
                                    elseif(j == 3)
                                        lamb = (L(i,1)*(LL3I(1,1)*L(j,1) + LL3I(1,2)*L(j,2)) + L(i,2)*(LL3I(2,1)*L(j,1) + LL3I(2,2)*L(j,2)))/(LL3I(1,1)*L(i,1)^2 + L(i,1)*L(i,2)*(LL3I(1,2) + LL3I(2,1)) + LL3I(2,2)*L(i,2)^2);
                                        tn = lamb*L(i,:);
                                        Dist(i,j) = sign(Gram(i,j))*norm(inv(LL3)*((L(j,:) - tn)'));
                                    end
                                    
                                end
                            end
                        end
                        % Determine the necessary angle for each side of the
                        % hyperplane
                        if((sum(Gram(i,:)>0)>1e-10)&&(sum(Gram(i,:)<-1e-10)>0))
                            temp_pos = max(Dist(i,:));
                            temp_neg = min(Dist(i,:));
                            Varsigma(i) = 0.2*sqrt(min(temp_neg^2,temp_pos^2));
                        else
                            pass = 0;
                        end
                        % Update semi_singular value if smaller than previous
                        i = i + 1;
                    end
                    if(pass == 1)
                        %                    Varsigma
                        %                    L
                        %                    Gram
                        Varsigma2 = ones(3,1);
                        for i =1:3
                            for j = 1:3
                                if(i==j)
                                    
                                else
                                    Varsigma2(i) = abs(Gram(j,i))*Varsigma2(i);
                                end
                            end
                        end
                        %                    Varsigma2
                        inWorkspace = 1;
                        semi_singular = 50*min(Varsigma)+2;
                        %                     semi_singular = 1;
                        disp('Yes')
                    else
                        inWorkspace = 0;
                        semi_singular = 0;
                        disp('no no no')
                    end
                end
            end
        end
        
        
        
        function [isConnected] = connected(obj,workspace,i,j,grid)
            % Connectiveness is evaluated using grid connectivity.
            % THIS FILE MAY NEED A DYNAMICS OBJECT ADDED AT A LATER DATE
            tol = 1e-6;
            isConnected = sum((abs(workspace(:,i) - workspace(:,j)) < grid.delta_q+tol)) + sum((abs(workspace(:,i) - workspace(:,j)-2*pi) < grid.delta_q+tol)) + sum((abs(workspace(:,i) - workspace(:,j)+2*pi) < grid.delta_q+tol)) == grid.n_dimensions;
        end
    end
end

