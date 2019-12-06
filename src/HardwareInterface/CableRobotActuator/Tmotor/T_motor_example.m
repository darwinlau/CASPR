



mymotor = TmotorCASPRInterface ('/dev/ttyACM0',2,0.000008,0.8,1/800,1/(2*pi),1);
mymotor.open;
mymotor.lengthInitialSend(10);

tic;

while(toc<10)
mymotor.lengthCommandSend(12);
end

mymotor.close;