% A measure of proximity to the unilateral singularity
% Author        : Jonathan EDEN
% Created       : 2016
% Description   : A separating hypersphere based metric
classdef SemiSingularMetric < WorkspaceMetricBase
    properties (SetAccess = protected, GetAccess = protected)
    end
    
    methods
        % Constructor
        function m = SemiSingularMetric()
        end
        
        % Evaluate Functions
        function v = evaluateFunction(obj,dynamics,~,method,inWorkspace)
            if((nargin <=3)||(~(method==WrenchClosureMethods.SS)))
                % Determine necessary variables for test
                L       =   dynamics.L; % Cable Jacobian
                L_rank  =   rank(L');   % Cable Jacobian Rank
                if(L_rank ~= dynamics.numDofs)
                    v = 0;
                else
                    numPlanes           =   nchoosek(dynamics.numCables,dynamics.numDofs - 1);
                    Gram                =   zeros(numPlanes,dynamics.numCables); % This matrix could also be used for separating hyperplane test
                    Varsigma            =   zeros(numPlanes,1);
                    % FOR THE MOMENT THIS IS ONLY WRITTEN FOR 2 AND 3D CASES,
                    % THE DIFFERENCE IS ONLY IN HANDLING COMBINATORICS
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
                        v = min(Varsigma);
                    else
                        v = 0;
                    end
                end
            else
                v = inWorkspace;
            end
        end
    end
end