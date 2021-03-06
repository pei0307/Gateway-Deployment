clear;
folder = 'data/SKfloorplan_modify/';
sub_folder = '20181228T204204/';
filename = [folder,sub_folder,'DeployResult.mat'];
load(filename);

floor_plan = [folder,'floorplan.png'];
floorPlan = imread(floor_plan);
floorPlanBW = ~im2bw(floorPlan);
originalFloorPlan = floorPlanBW; % At this point, Structure in Original Image and floorPlanBW is in black
load (Pixel_Setting) ;
load (Pathloss_Distance);

% [lossdB,User_Served,User_Covered,User_Arc] = Deploy_Result3(GW_Pathloss_perPixel,Tx_ind,Rxr,Rxc,-84.4505,GW_Serve_Limit);
period = 1000;
%% initialization1
meanGServednum = [];
FInishRate = [];
WorstFinish = [];

Fail_ind = zeros(1,size(Rxc,1));
Access_ind = zeros(1,size(Rxc,1));
[lossdB,User_Served,User_Covered,User_Arc] = Deploy_Result3(GW_Pathloss_perPixel,Tx_ind,Rxr,Rxc,-84.4505,GW_Serve_Limit);
Result = [];
Num_Record = zeros(1,period);
for r =1:1:period
    User = zeros(size(Rxc,1),4);
    User_num = 0;
    for i=1:1:size(Rxc,1)
        if rand() < Density_map(i)
            User_num = User_num+1;
            User(User_num,1) = i;
            User(User_num,2) = randi([0,359]);
        end
    end
    Num_Record(r) = User_num;
    % User = 1:path /2:angle record /3:Tx index /4:Rx_power
    %Result = 1:Fail rate/ 2.mean Rx power/3:success user num
    Tx_Result =[];
    step = 100;
    for t = 1 : 1 : step
        [User,Tx_Record,Fail_ind,Access_ind] = GW_Assignment(User,Rxc,Rxr,User_num,User_Covered,User_Arc,GW_Serve_Limit,Fail_ind,Access_ind);
        Result = [Result;length(find(User(1:User_num,3)==0))/User_num , sum(User(1:User_num,4))/length(find(User(1:User_num,3)~=0)) , length(find(User(1:User_num,3)~=0))];
        Tx_Result = [Tx_Result;Tx_Record(find(Tx_ind>0))];
        User = Random_walk(User,User_num,meshNode,Distance_perPixel);
    end
end

User_mean = mean(Num_Record);

Period_Result=zeros(1,period);
for i=1:1:period
    Period_Result(i) = mean(Result((i-1)*step+1:(i*step),1));
end

mean_Rxpower = mean(Result(:,2));
mean_GWserved = mean(Result(:,3)/length(find(Tx_ind==1)));
mean_Tx = mean(Tx_Result)
Finish_rate = 1-mean(Result(:,1));
Worst_case = 1-max(Period_Result);

fprintf('Mean User Num : %f \n',User_mean);
fprintf('Tx Num  :\n Proposed:%d \n',length(find(Tx_ind==1)));
fprintf('Mean Rx Power :\n Proposed:%f \n',mean_Rxpower);
fprintf('Mean GW Served :\n Proposed:%f \n',mean_GWserved);
fprintf('Finish rate :\n Proposed:%f \n',Finish_rate);
fprintf('Worst_case :\n Proposed:%f \n',Worst_case);
sfilename = [filename(1:length(filename)-4),'_SimulationResult.mat'];
save(sfilename,'Tx_ind','Num_Record','Result','Period_Result','Finish_rate','Worst_case','Tx_Result','Fail_ind','Access_ind','period','step','floorPlan');
meanGServednum = [meanGServednum;mean_GWserved];
FInishRate = [FInishRate;Finish_rate];
WorstFinish = [WorstFinish;Worst_case];



%%f1
figure('Name',['Path loss method ']);
imshow(floorPlan);
% text(Rxc,Rxr,num2str(int32(lossdB)'),'FontSize',8);
% text(Rxc(Tx_ind==1),Rxr(Tx_ind==1),num2str(round(mean(Tx_Result))'),'FontSize',10);
text(Rxc(Tx_ind==1),Rxr(Tx_ind==1),'*','Color','Black','FontSize',20);
Tx = find(Tx_ind==1);
% for i=1:1:Tx_Num
%     if mean_Tx(i) <= 3.5
%         text(Rxc(Tx(i)),Rxr(Tx(i)),num2str(round(mean_Tx(i))),'Color','red','FontSize',15);
%     else
%         text(Rxc(Tx(i)),Rxr(Tx(i)),num2str(round(mean_Tx(i))),'Color','black','FontSize',15);
%     end
% end
title([' GW num = ',num2str(length(find(Tx_ind==1))),],'FontSize',15);
figure;
hist(round(mean(Tx_Result)),0:1:6)

