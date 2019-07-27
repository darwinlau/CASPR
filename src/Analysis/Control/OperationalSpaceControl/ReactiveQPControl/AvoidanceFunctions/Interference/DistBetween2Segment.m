% Computes the minimum distance between two line segments. Code
% is adapted for Matlab from Dan Sunday's Geometry Algorithms originally
% written in C++
% http://softsurfer.com/Archive/algorithm_0106/algorithm_0106.htm#dist3D_Segment_to_Segment

% Usage: Input the start and end x,y,z coordinates for two line segments. 
% p1, p2 are [x,y,z] coordinates of first line segment and p3,p4 are for
% second line segment. 

% Output: scalar minimum distance between the two segments.

%  Example:
%	P1 = [0 0 0];     P2 = [1 0 0];
%   P3 = [0 1 0];     P4 = [1 1 0];
%	dist = DistBetween2Segment(P1, P2, P3, P4)
%   dist =
%
%    1
% 


%{ 
Senario flags
Cable           Link
1 segment       segment
2 segment       Tip start
3 segment       Tip end
4 Tip start     segment
5 Tip end       segment
6 Tip start     Tip start
7 Tip end       Tip end
8 Tip start     Tip end
9 Tip end       Tip start
%}

function [distance, varargout] = DistBetween2Segment(p1, p2, p3, p4)
    flag = 1;
    u = p1 - p2;
    v = p3 - p4;
    w = p2 - p4;
    
    a = dot(u,u);
    b = dot(u,v);
    c = dot(v,v);
    d = dot(u,w);
    e = dot(v,w);
    D = a*c - b*b;
    sD = D;
    tD = D;
    
    SMALL_NUM = 0.00000001;
    
    % compute the line parameters of the two closest points
    if (D < SMALL_NUM)  % the lines are almost parallel
        sN = 0.0;       % force using point P0 on segment S1
        sD = 1.0;       % to prevent possible division by 0.0 later
        tN = e;
        tD = c;
    else                % get the closest points on the infinite lines TO CHECK IF THEY ARE INSIDE SHORT D?
        sN = (b*e - c*d);   
        tN = (a*e - b*d);
        if (sN < 0.0)   % sc < 0 => the s=0 edge is visible       % Scenario 5
            sN = 0.0;
            tN = e;
            tD = c;
            flag = 5;
        elseif (sN > sD)% sc > 1 => the s=1 edge is visible  % Scenario 4
            sN = sD;
            tN = e + b;
            tD = c;
            flag = 4;
        end
    end
    
    if (tN < 0.0)            % tc < 0 => the t = 0 edge is visible
        tN = 0.0;
        % recompute sc for this edge
        if (-d < 0.0) 
            sN = 0.0;  % Scenario 7
            flag = 7;
        elseif (-d > a)
            sN = sD; % Scenario 8
            flag = 8;            
        else
            sN = -d; %Scenario 3
            sD = a;
            flag = 3;
        end
    elseif (tN > tD)       % tc > 1 => the t=1 edge is visible
        tN = tD;
        % recompute sc for this edge
        if ((-d + b) < 0.0) %Scenario 9
            sN = 0;
            flag = 9;
        elseif ((-d + b) > a) % Scenario 6
            sN = sD;
            flag = 6;            
        else 
            sN = (-d + b);  % Scenario 2
            sD = a;
            flag = 2;
        end
    end
    
    % finally do the division to get sc and tc
    if(abs(sN) < SMALL_NUM)
        sc = 0.0;
    else
        sc = sN / sD;
    end
    
    if(abs(tN) < SMALL_NUM)
        tc = 0.0;
    else
        tc = tN / tD;
    end
    
    % get the difference of the two closest points
    dP = w + (sc * u) - (tc * v);  % = S1(sc) - S2(tc)

    distance = norm(dP);
    outV = dP;

    varargout(1) = {flag};
    varargout(2) = {outV};      % vector connecting the closest points
    varargout(3) = {p2+sc*u};   % Closest point on object 1
    varargout(4) = {p4+tc*v};   % Closest point on object 2    
%     fprintf('Scenario: %d\n', flag);
end