clc;clear;
exp = PoCaBotExperiment(1);
exp.motorSpoolTest();

% acc = MotorAccessories;
% acc.setInitState(0.2);
% deltaLength = -1.8:0.1:0;
% for i = 1:length(deltaLength)
%     deltaAngle(i) = acc.getDeltaAngle(deltaLength(i),0);
%     deltaAngle_test(i) = acc.getDeltaAngle(deltaLength(i),1);
% end
% 
% plot(deltaLength,deltaAngle,'b');
% hold on; grid on;
% plot(deltaLength,deltaAngle_test,'r');
% plot(deltaLength,(deltaAngle_test-deltaAngle)*180/pi,'*');
% legend('Modeled','Linearized');

