function dataOut = saveToCSV2(nstep,interval,Time,x,y,z,pth,yaw,rll,delta)
NN = 1:interval:nstep;
Index = 1:length(NN);
dataOut2 = [Index',Time(NN)',x(NN),y(NN),z(NN),pth(NN),yaw(NN),rll(NN),delta(NN)];

title = {'Null','Time','x','y','z','pth','yaw','rll','delta'};
dataOut = table(dataOut2(:,1),dataOut2(:,2),dataOut2(:,3),dataOut2(:,4),dataOut2(:,5),dataOut2(:,6),dataOut2(:,7),dataOut2(:,8),dataOut2(:,9),'VariableNames',title);

writetable(dataOut,'dataOut3.csv');
writetable(dataOut,'D:\data\Unreal Projects\CarPostProcessor0804\PackUp\PackUp0817\WindowsNoEditor\CarPostProcessor0804\Content\BlueprintData\dataOut3.csv');

