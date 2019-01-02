clear;
folder = 'data/SKroom/';
sub_folder = '';
filename = [folder,sub_folder,'DeployResult.mat'];
% load(filename);

floor_plan = [folder,'floorplan.jpg'];

floorPlan = imread(floor_plan);
floorPlanBW = ~im2bw(floorPlan);
originalFloorPlan = floorPlanBW; % At this point, Structure in Original Image and floorPlanBW is in black

wall_detect = [folder,'wall_detect.mat'];
Pixel_Setting = [folder, 'pixel_Setting.mat'];
Pathloss_Distance = [folder, 'PlandDis_perpixel'];
sfilename = [filename(1:length(filename)-4),'_SimulationResult_2.mat'];
GW_Serve_Limit = 2;

load (Pixel_Setting) ;
load (Pathloss_Distance);
load(wall_detect);

period = 1000;
%% initialization1

% 自定義GW位置
Tx_ind = zeros(1,size(Rxc,1));
% Tx_ind([1,18])=1;
Tx_ind([3,17])=1;
% Tx_ind([3,18])=1;

Fail_ind = zeros(1,size(Rxc,1));
Access_ind = zeros(1,size(Rxc,1));
[lossdB,User_Served,User_Covered,User_Arc] = Deploy_Result3(GW_Pathloss_perPixel,Tx_ind,Rxr,Rxc,-84.4505,GW_Serve_Limit);
Result = [];
Num_Record = zeros(1,period);
for r =1:1:period
%     自定義user個數
    User_num = 2;
    User = zeros(size(Rxc,1),4);
    for i=1:1:User_num
        User(i,1) = randi([1,size(Rxc,1)]);
        User(i,2) = randi([0,359]);
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
load(sfilename);
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

% save(sfilename,'Tx_ind','Num_Record','Result','Period_Result','Finish_rate','Worst_case','Tx_Result','Fail_ind','Access_ind','period','step','floorPlan');

fprintf('Mean User Num : %f \n',User_mean);
fprintf('Tx Num  :\n Proposed:%d \n',length(find(Tx_ind==1)));
fprintf('Mean Rx Power :\n Proposed:%f \n',mean_Rxpower);
fprintf('Mean GW Served :\n Proposed:%f \n',mean_GWserved);
fprintf('Finish rate :\n Proposed:%f \n',Finish_rate);
fprintf('Worst_case :\n Proposed:%f \n',Worst_case);

floorPlanBW = im2bw(floorPlan);
figure('Name',['Path loss method ']);
imshow(~imdilate(~floorPlanBW,strel('disk',2)));
text(Rxc,Rxr,num2str(Fail_ind'),'FontSize',15);
% text(Rxc(Tx_ind==1),Rxr(Tx_ind==1),num2str(round(mean(Tx_Result))'),'FontSize',10);
text(Rxc(Tx_ind==1),Rxr(Tx_ind==1),'*','Color','Black','FontSize',20);
title([' GW num = ',num2str(length(find(Tx_ind==1))),],'FontSize',15);

figure;
hist(round(mean(Tx_Result)),0:1:6)

