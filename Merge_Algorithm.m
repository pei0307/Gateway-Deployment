function [ lossdB,Tx_ind,User_Covered,User_Served,User_Arc,GW_Num ] = Merge_Algorithm(folder,merge_method,Result_filename,Range)

%%
%2018/5/11
%find every point's shortest distance point as candidate 9

BER2SNR = @(BER)((erfcinv(2*BER))^2)*2;
SNR_Thres = BER2SNR(10^(-3));
Temperature = 300;
Bandwidth = 2*10^(6);
noise = -94; %dbm
Thermal_noise = 10*log10(1.38*10^(-23)*Temperature*Bandwidth*1000);
TxP_Thres = SNR_Thres + noise;
Channel = 37;

Pixel_Setting = [folder,'pixel_Setting.mat'];
Pathloss_Distance = [folder,'PlandDis_perpixel.mat'];

load (Pixel_Setting) ;
load (Pathloss_Distance) ;
load (Result_filename);

GW_Num = length(find(Tx_ind==1));

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

for t = 1:1:GW_Num
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
    Upper_U = -10000;
    % consider all the link, choose the highest utility one to merge
    for k = 1:1:size(All_Link,1)
        fprintf('t = %d k = %d\n',t,k);
        Temp_Tx_ind = Tx_ind;
        Temp_Tx_ind(Txind_to_Rxind(All_Link(k,1))) = 0;
        Temp_Tx_ind(Txind_to_Rxind(All_Link(k,2))) = 0;
        
        Candidate_Pos1 = CoverRange_perPixel(Txind_to_Rxind(All_Link(k,1)),:);
        Candidate_Pos2 = CoverRange_perPixel(Txind_to_Rxind(All_Link(k,2)),:);
        Candidate_Pos = find((Candidate_Pos1 & Candidate_Pos2)==1);

        Cadidate_ind = zeros(1,size(Rxc,1));
        Cadidate_Num = 0;
        for j=1:1:length(Candidate_Pos)
            if Temp_Tx_ind(Candidate_Pos(j)) == 0
                Cadidate_ind(Candidate_Pos(j)) = 1;
                Cadidate_Num = Cadidate_Num+1;
            end
        end
        Candidate_Pos = find(Cadidate_ind == 1);
        
        [try_lossdB,try_User_Served,try_User_Covered,try_User_Arc] = Deploy_Result(GW_Pathloss_perPixel,CoverRange_perPixel,Temp_Tx_ind,Rxr,Rxc,TxP_Thres,GW_Serve_Limit,Density_map);
        U = -10000;
        Have_local_best = 0;
        for j =1:1:Cadidate_Num
            Temp2_Tx_ind = Temp_Tx_ind;
            Temp2_Tx_ind(Candidate_Pos(j)) =1;
            [try2_lossdB,try2_User_Served,try2_User_Covered,try2_User_Arc] = Put_OneGW(GW_Pathloss_perPixel,CoverRange_perPixel,Candidate_Pos(j),Rxc,Rxr,try_User_Served,try_User_Arc,try_User_Covered,try_lossdB,TxP_Thres,GW_Serve_Limit,Density_map);
            Fitness = Constraint_Fulfilled([1:1:size(Rxc,1)],try2_User_Served,try2_User_Arc);
            try_Tx_Record = zeros(1,size(Rxc,1));
            for i =1:1:size(Rxr,1)
                try_Tx_Record(i) = size(find(try2_User_Served==i),1);
            end

            Utility = try_Tx_Record(Candidate_Pos(j));

            if (Fitness == 1) && (Utility > U)
                U = Utility;
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
        if Have_local_best && (U > Upper_U)
            Upper_U = U;
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
    else
        break;
    end
    
    
end

%% Applying color map
% figure;
% imshow(floor_plan);
% for i=1:1:size(Rxr,1)
%     if Tx_ind(i) == 1
%         text(Rxc(i),Rxr(i),'*','Color','Black','FontSize',20);
%     end
% end
% text(Rxc(Candidate_Pos),Rxr(Candidate_Pos),'o','Color','red','FontSize',10);
% text(Rxc(Txind_to_Rxind(All_Link(k,1))),Rxr(Txind_to_Rxind(All_Link(k,1))),'*','Color','red','FontSize',30);
% text(Rxc(Txind_to_Rxind(All_Link(k,2))),Rxr(Txind_to_Rxind(All_Link(k,2))),'*','Color','red','FontSize',30);

%%%%%%%%%%%%%%%%5  REFERENCES  %%%%%%%%%%
% http://uk.mathworks.com/matlabcentral/fileexchange/28190-bresenham-optimized-for-matlab/content/bresenham.m