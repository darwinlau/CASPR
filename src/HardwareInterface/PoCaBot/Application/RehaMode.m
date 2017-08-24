% This script is for rehabilitation application
if ~exist('exp','var')
    exp = PoCaBotExperiment(8,'dualcables_cylinder_handle');
    exp.rehabilitation_preparation();
end

duration = 300;%seconds
exp.rehabilitation_run(duration);
% exp.application_termination();
% clear exp;
fprintf('Construction Finished.\n');