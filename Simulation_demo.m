clear;

floor_plan = 'data\thesis.png' ;
floorPlan = imread(floor_plan);
floorPlanBW = ~im2bw(floorPlan);
% adjusting the image if required
try
    [co,bi] = imhist(floorPlanBW);
    
    if bi(1) == 0 && co(1) > co(2)
        floorPlanBW = ~floorPlanBW; % complements the image if not in correct form so that the structure will be in Black in case it's not
    end
catch
end
originalFloorPlan = floorPlanBW; % At this point, Structure in Original Image and floorPlanBW is in black
GW_Serve_Limit =6;
Pixel_Setting = [floor_plan(1:length(floor_plan)-4),'_pixel_Setting.mat'];
Pathloss_Distance = [floor_plan(1:length(floor_plan)-4),'_PlandDis_perpixel.mat'];
load (Pixel_Setting) ;
load (Pathloss_Distance);

% [lossdB,User_Served,User_Covered,User_Arc] = Deploy_Result3(GW_Pathloss_perPixel,Tx_ind,Rxr,Rxc,-84.4505,GW_Serve_Limit);
period = 10000;
%% initialization1
meanGServednum = [];
FInishRate = [];
WorstFinish = [];

    filename1 = [floor_plan(1:length(floor_plan)-4),'_flexibleDensity.mat'];
    load(filename1);
    [lossdB,User_Served,User_Covered,User_Arc] = Deploy_Result3(GW_Pathloss_perPixel,Tx_ind,Rxr,Rxc,-84.4505,GW_Serve_Limit);
    GW_Serve_Limit=6;
    Result = [];
    Num_Record = zeros(100,1);
    for r =1:1:100
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
        % User = 1:path /2:angle record
        %Result
        Tx_Result =[];
        for t = 1 : 1 : period
            [User,Tx_Record] = GW_Assignment(User,Rxc,Rxr,User_num,User_Covered,User_Arc,GW_Serve_Limit);
            Result = [Result;length(find(User(1:User_num,3)==0))/User_num,sum(User(1:User_num,4))/length(find(User(1:User_num,3)~=0)),length(find(User(1:User_num,3)~=0))];
            Tx_Result = [Tx_Result;Tx_Record(find(Tx_ind>0))];
            User = Random_walk(User,User_num,meshNode);
        end
    end

    User_mean = mean(Num_Record);
    
    Period_Result=zeros(1,1000);
    for i=1:1:period
        Period_Result(i) = mean(Result((i-1)*100+1:(i*100),1));
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
     sfilename = [floor_plan(1:length(floor_plan)-4),'_SimulationResult_Fealsible'];
     save(sfilename,'Num_Record','Result','Period_Result','Finish_rate','Worst_case','Tx_Result');
    meanGServednum = [meanGServednum;mean_GWserved];
    FInishRate = [FInishRate;Finish_rate];
    WorstFinish = [WorstFinish;Worst_case];



originalFloorPlan = imread(floor_plan);
originalFloorPlan = im2bw(originalFloorPlan);
originalFloorPlan =  ~imdilate(~originalFloorPlan,strel('disk',2));
smallFSPLImage = (reshape(lossdB,meshNode.vert.num, meshNode.horz.num));
FSPLFullImage = (imresize(smallFSPLImage,[size(originalFloorPlan,1),size(originalFloorPlan,2)],'method','cubic'));
FSPLFullImage = mat2gray(FSPLFullImage);
figure('Name',['Path loss method ']);
z = imoverlay(FSPLFullImage,~originalFloorPlan,[0,0,0]);
imshow(rgb2gray(z));
text(Rxc,Rxr,num2str(int32(lossdB)'),'FontSize',8);
D=3;
mean_Tx = mean(Tx_Result);
Tx_Num = length(find(Tx_ind==1));

%%f1
figure('Name',['Path loss method ']);
imshow(~imdilate(~originalFloorPlan,strel('disk',2)));
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
