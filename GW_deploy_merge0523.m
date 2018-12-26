%%
%2018/5/11
%find every point's shortest distance point as candidate 9
%% INITIALIZATION  (Change them to your liking) ///////////////////////////////////////////////////////
clear all
clc

lightVel                = 3e8;      % Light velocity (m/s).
freq                    = 865.2e6;  % Hz

% attendB                 = 1;        % 1 if attenuation in dB, other wise linear
pathLossModel           = 2;        % 1=Free-Space, 2=Motley-Keenan (COST231)
choose_option           = 1;
demoMode                = 0;        % Showes furtther details if 1
calibrate_option        = 1;
merge_method            = 0;

TxPower                 = -63.8;        % dBm or dB
antennaLoss             = 0;        % dB
TxAntennaGain           = 0 + antennaLoss ; % Gain of Transmitting antenna
RxAntennaGain           = TxAntennaGain;    % Gain of Receiving antenna

Pixel_Res = 1;    %pixel=2m*2m
% meshNode.vert.num       = 40;             % Number of probs in the structure increase for better accuracy
% meshNode.horz.num       = 40;

% Free Space Path Loss Model Parameters
d0FSPL                  = 1;      % Reference Distance (meters) for Free Space Path Loss model

% Multi-Wall Model Parameters
d0Cost231               = 1;      % Multi-wall model reference distance

% Wall Detection Parameters (Change them wiselt if walls are not correctly detected)
thetaRes = 0.1;                   % Resolution of the Hough Transform Space (don't make it smaller than 0.1)
minWallLength = 5;               % Minimum length of the walls in pixel
fillGap = 3;                     % Gap between walls

%Wall Attenuation Coefficient Assignment dB
% Manually Assign Attenuation factors to each particular wall based on it's
% intensity dynamic range (change plot mode in autoWallDetection to 1 to
% see the intesity of each wall.
wallAt = zeros(255,1);
wallAt(1:end) = 10;
wallAt(255) = round(sum(wallAt)./sum(wallAt>0));  % This is for intersecting walls, just leave it as it is

BER2SNR = @(BER)((erfcinv(2*BER))^2)*2;
SNR_Thres = BER2SNR(10^(-3));
Temperature = 300;
Bandwidth = 2*10^(6);
noise = -94; %dbm
Thermal_noise = 10*log10(1.38*10^(-23)*Temperature*Bandwidth*1000);
TxP_Thres = SNR_Thres + noise;
Channel = 37;
Collision_Prob = @(n) 1-(1-1/Channel)^n;

Data_Rate = (2*(256)*8)/1000; % kbps
Max_throughput = 113; %kbps(BER=10^-3)
Density = 0.0625;
GW_Serve_Limit = floor((Max_throughput*(1000-294.7580)/1000)/Data_Rate/3)/Density;

PdBm_to_Pw = @(PdBm) 10^(PdBm/10);
Pw_to_PdBm = @(Pw) 10*log10(Pw);
Utility_func = @(P_r,P_int) log2(1+P_r/P_int);
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %/////////////////////////////////////////////////////////////////////////
%
% % Reading the image
% % try
% %     [fileName,filePath] = uigetfile('*.*');
% % catch
% % end
% floorPlan = imread([filePath,fileName]);

floor_plan = 'data\SKfloorplan_modify.png';
wall_detect = [floor_plan(1:length(floor_plan)-4),'_wall_detect.mat'];
Pixel_Setting = [floor_plan(1:length(floor_plan)-4),'_pixel_Setting.mat'];
Pathloss_Distance = [floor_plan(1:length(floor_plan)-4),'_PlandDis_perpixel.mat'];
DeployResult = [floor_plan(1:length(floor_plan)-4),'_flexibleDensity0.06.mat'];
load (wall_detect) ;
load (Pixel_Setting) ;
load (Pathloss_Distance) ;
load (DeployResult);

U=[];
Utility_Option = 1;
Txc = Rxc;
Txr = Rxr;
Tx_Record = Tx_ind;
GW_Num = length(find(Tx_ind==1));
Range = 4;
%% Calculate Covering range
CoverRange_perPixel = zeros(size(Rxc,1),size(Rxc,1));
for i=1:1:size(Rxc,1)
    Covered1_P = zeros(1,size(Rxc,1));
    Covered2_P =zeros(1,size(Rxc,1));
    [Sort_P1,ind_P1] = sort(GW_Pathloss_perPixel(i,:),'descend');
    ind_P1 = ind_P1(GW_Pathloss_perPixel(i,ind_P1)>=TxP_Thres);
    Covered1_P = Calculate_Range(Density_map,Sort_P1,ind_P1,GW_Serve_Limit*Range);
    
    [Sort_DP,ind_DP] = sort(Distance_perPixel(i,:));
    ind_DP = ind_DP(GW_Pathloss_perPixel(i,ind_DP)>=TxP_Thres);
    Covered2_P = Calculate_Range(Density_map,Sort_DP,ind_DP,GW_Serve_Limit*Range);
    
    Covered_P = (Covered1_P |Covered2_P);
    CoverRange_perPixel(i,:) = Covered_P;
end
for i=1:1:size(Rxc,1)
    CoverRange_perPixel(i,:) =((CoverRange_perPixel(i,:) ) | (CoverRange_perPixel(:,i)'));
end


for t = 1:1:size(Rxr,1)
    
    Txc = Rxc(Tx_ind == 1);
    Txr = Rxr(Tx_ind == 1);
    Txind_to_Rxind = find(Tx_ind == 1);
    All_Link =[];
    % find shortest distance as link
    if merge_method == 0
        Temp_Txc = [];
        Temp_Txr = [];
        for k =1:1: length(Txc)
            Temp_Txc(k,:) = (Txc(k)-Txc(:))';
            Temp_Txr(k,:) = (Txr(k)-Txr(:))';
        end
        distances = sqrt((Temp_Txc).^2 + (Temp_Txr).^2);
        for k=1:1:length(Txc)
            distances(k,k) = 3000;
            Y = min(distances(k,:));
            ind = find(distances(k,:) == Y);
            for j=1:1:length(ind)
                if GW_Pathloss_perPixel(k,ind(j)) >= TxP_Thres
                    All_Link = [All_Link ; sort([k,ind(j)],2)];
                end
            end
        end
        % construct delauney tri as link
    elseif merge_method == 1
        tri = delaunay(Txc,Txr);
        for k=1:1:size(tri,1)
            All_Link = [All_Link ; sort([tri(k,1) tri(k,2)],2)];
            All_Link = [All_Link ; sort([tri(k,2) tri(k,3)],2)];
            All_Link = [All_Link ; sort([tri(k,3) tri(k,1)],2)];
        end
    end
    All_Link = unique(All_Link(:,1:2),'rows');
    Have_global_best = 0;
    Upper_U2 = -10000;
    Upper_U3 = -10000;
    % consider all the link, choose the highest utility one to merge
    for k = 1:1:size(All_Link,1)
        fprintf('t = %d k = %d\n',t,k);
        Temp_Tx_ind = Tx_ind;
        Temp_Tx_ind(Txind_to_Rxind(All_Link(k,1))) = 0;
        Temp_Tx_ind(Txind_to_Rxind(All_Link(k,2))) = 0;
        
        Candidate_Pos = find(GW_Pathloss_perPixel(Txind_to_Rxind(All_Link(k,1)),:) > TxP_Thres);
        Candidate_Pos = [Candidate_Pos , find(GW_Pathloss_perPixel(Txind_to_Rxind(All_Link(k,2)),:) > TxP_Thres)];
        Candidate_Pos = unique(Candidate_Pos);
        
        Cadidate_ind = zeros(1,size(Rxc,1));
        Cadidate_Num = 0;
        for j=1:1:length(Candidate_Pos)
            if Temp_Tx_ind(Candidate_Pos(j)) == 0
                Cadidate_ind(Candidate_Pos(j)) = 1;
                Cadidate_Num = Cadidate_Num+1;
            end
        end
        Utility_perPixel = zeros(1,size(Rxc,1));
        Pinterf_perPixel = zeros(1,size(Rxc,1));
        Tx_Collision_Num = zeros(1,size(Rxc,1));
        Tx_Utility = zeros(1,size(Rxc,1));
        for j=1:1:size(Rxc,1)
            for l = 1:length(Txind_to_Rxind)
                if GW_Pathloss_perPixel(j,Txind_to_Rxind(l)) > -85 && j ~= Txind_to_Rxind(l)
                    Tx_Collision_Num = Tx_Collision_Num + 1;
                    Pinterf_perPixel(j) = Pinterf_perPixel(j) + 10^(GW_Pathloss_perPixel(j,Txind_to_Rxind(l))/10);
                end
            end
            %                 Tx_Utility(j) = Tx_Record(j)* log2(1+(10^(mean(User_Served(find(User_Served(:,1)==j),2))/10))/Tx_Pinterf(j));
            Utility_perPixel(j) = Utility_func(PdBm_to_Pw(User_Served(j,2)),Pinterf_perPixel(j));
            Tx_Utility(User_Served(j,1)) = Tx_Utility(User_Served(j,1)) + Utility_perPixel(j);
        end
        Candidate_Pos = find(Cadidate_ind == 1);
        %             [try_lossdB,try_User_Served,try_User_Covered,try_User_Arc,try_GW_Pathloss_perPixel]= Calculate_Pathloss2(originalFloorPlan,floorPlanGray,wallAt,pathLossModel,Temp_Tx_ind,Rxc,Rxr,pathUnit);
        [try_lossdB,try_User_Served,try_User_Covered,try_User_Arc] = Deploy_Result3(GW_Pathloss_perPixel,Temp_Tx_ind,Rxr,Rxc,TxP_Thres,GW_Serve_Limit);
        U2 = -10000;
        U3 = -10000;
        Have_local_best = 0;
        for j =1:1:Cadidate_Num
            Temp2_Tx_ind = Temp_Tx_ind;
            Temp2_Tx_ind(Candidate_Pos(j)) =1;
            %                 [try2_lossdB,try2_User_Served,try2_User_Covered,try2_User_Arc] = Deploy_Result(GW_Pathloss_perPixel,Temp2_Tx_ind,Rxr,Rxc,TxP_Thres);
            [try2_lossdB,try2_User_Served,try2_User_Covered,try2_User_Arc] = Put_OneGW(GW_Pathloss_perPixel,CoverRange_perPixel,Candidate_Pos(j),Rxc,Rxr,User_Served,User_Arc,User_Covered,lossdB,TxP_Thres,GW_Serve_Limit,Density_map);
            %                 [try2_lossdB,try2_User_Served,try2_User_Covered,try2_User_Arc] = Put_OneGW(originalFloorPlan,floorPlanGray,wallAt,pathLossModel,Candidate_Pos(j),Rxc,Rxr,pathUnit,try_User_Served,try_User_Arc,try_User_Covered,try_lossdB);
            Fitness = Constraint_Fulfilled([1:1:size(Rxc,1)],try2_User_Served,try2_User_Arc);
            try_Tx_Record = zeros(1,size(Rxc,1));
            for i =1:1:size(Rxr,1)
                try_Tx_Record(i) = size(find(try2_User_Served==i),1);
            end
            %             F3 = length(find(try_Tx_Record(:) <= GW_Serve_Limit))/length(try_Tx_Record)
            collision_num =0;
            Temp_Txind_to_Rxind = find(Temp_Tx_ind == 1);
            P_interf = 0;
            for i = 1:length(Temp_Txind_to_Rxind)
                if GW_Pathloss_perPixel(Candidate_Pos(j),Temp_Txind_to_Rxind(i)) > -85 && Candidate_Pos(j) ~= Temp_Txind_to_Rxind(i)
                    collision_num = collision_num+1;
                    P_interf = P_interf + 10^(GW_Pathloss_perPixel(Candidate_Pos(j),Temp_Txind_to_Rxind(i))/10);
                end
            end
            Utility_2 = 0;
            Served_ID = find(try2_User_Served(:,1)==Candidate_Pos(j));
            for i=1:try_Tx_Record(Candidate_Pos(j))
                Utility_2 = Utility_2 + Utility_func(PdBm_to_Pw(try2_User_Served(Served_ID(i),2)),P_interf);
            end
            if Utility_Option == 1
                Utility_3 = try_Tx_Record(Candidate_Pos(j));
            elseif Utility_Option == 2
                Utility_3 = mean([Tx_Utility(Txind_to_Rxind(All_Link(k,1))),Tx_Utility(Txind_to_Rxind(All_Link(k,2)))])- Utility_2;
            elseif Utility_Option == 3
                Utility_3 = mean([Tx_Utility(Txind_to_Rxind(All_Link(k,1)))/Tx_Record(Txind_to_Rxind(All_Link(k,1))),Tx_Utility(Txind_to_Rxind(All_Link(k,2)))/Tx_Record(Txind_to_Rxind(All_Link(k,1)))])- Utility_2/try_Tx_Record(Candidate_Pos(j));
            end
            if (Fitness == 1) && (Utility_3 > U3)
                U3 = Utility_3;
                U2 = Utility_2;
                Local_Tx_ind = Temp_Tx_ind;
                Local_Tx_ind(Candidate_Pos(j)) = 1;
                Local_lossdB = try2_lossdB;
                Local_User_Served = try2_User_Served;
                Local_User_Covered = try2_User_Covered;
                Local_User_Arc = try2_User_Arc;
                Local_Tx_Record = try_Tx_Record;
                Have_local_best = 1;
            end
        end
        if Have_local_best && (U3 > Upper_U3)
            Upper_U3 = U3;
            Upper_U2 = U2;
            Best_Tx_ind = Local_Tx_ind;
            Best_lossdB = Local_lossdB;
            Best_User_Served = Local_User_Served;
            Best_User_Covered = Local_User_Covered;
            Best_User_Arc = Local_User_Arc;
            Best_Tx_Record = Local_Tx_Record;
            Have_global_best = 1;
        end
    end
    if Have_global_best == 1
        Tx_ind = Best_Tx_ind;
        lossdB = Best_lossdB;
        User_Served = Best_User_Served;
        User_Covered = Best_User_Covered;
        User_Arc = Best_User_Arc;
        Tx_Record = Best_Tx_Record;
        GW_Num = length(find(Tx_ind == 1));
        fprintf('GW Num = %d\n',GW_Num);
        U = [U,sum(Tx_Utility)]
    else
        break;
    end
    
    
end
filename = ['small4_D',num2str(den),'_0625'];
save(filename,'Density','lossdB','Tx_ind','Tx_Record','Tx_Utility','User_Covered','User_Served','User_Arc','U','GW_Serve_Limit');
%% Applying color map
smallFSPLImage = (reshape(lossdB,meshNode.vert.num, meshNode.horz.num));
FSPLFullImage = (imresize(smallFSPLImage,[size(floorPlan,1),size(floorPlan,2)],'method','cubic'));
FSPLFullImage = mat2gray(FSPLFullImage);
figure('Name',['Path loss method ']);
z = imoverlay(FSPLFullImage,~originalFloorPlan,[0,0,0]);
imshow(rgb2gray(z));
% text(Rxc,Rxr,num2str(int32(nodeDistance)),'FontSize',7);
% text(Rxc,Rxr,num2str([1:1:size(Rxc,1)]'),'FontSize',7);
text(Rxc,Rxr,num2str(User_Served(:,1)),'FontSize',10);
colormap(gca,'jet');

for i = 1:7
    colorbarLabels(i) = min(lossdB) + i .* ((max(lossdB)-min(lossdB))./7);
end
colorbar('YTickLabel',num2str(int32(colorbarLabels')));
for i=1:1:size(Rxr,1)
    if Tx_ind(i) == 1
        text(Rxc(i),Rxr(i),'*','Color','Black','FontSize',30);
    end
end
title(['small4 D',num2str(den),' U1 GW',num2str(GW_Num)]);


%%%%%%%%%%%%%%%%5  REFERENCES  %%%%%%%%%%
% http://uk.mathworks.com/matlabcentral/fileexchange/28190-bresenham-optimized-for-matlab/content/bresenham.m