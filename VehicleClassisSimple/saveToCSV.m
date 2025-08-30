function dataOut = saveToCSV(nstep,interval,Time,dx,dy,dz,dpth,dyaw,drll)
NN = 1:interval:nstep;
Index = 1:length(NN);
dataOut2 = [Index',Time(NN)',dx(NN),dy(NN),dz(NN),dpth(NN),dyaw(NN),drll(NN)];

title = {'Null','Time','dx','dy','dz','dpth','dyaw','drll'};
dataOut = table(dataOut2(:,1),dataOut2(:,2),dataOut2(:,3),dataOut2(:,4),dataOut2(:,5),dataOut2(:,6),dataOut2(:,7),dataOut2(:,8),'VariableNames',title);

writetable(dataOut,'dataOut.csv');