Cur_P = 1000;
    Covered1_P = zeros(1,size(Rxc,1));
    Covered2_P =zeros(1,size(Rxc,1));
    [Sort_P1,ind_P1] = sort(GW_Pathloss_perPixel(Cur_P,:),'descend');
    ind_P1 = ind_P1(GW_Pathloss_perPixel(Cur_P,ind_P1)>=TxP_Thres);
    Covered1_P = Calculate_Range(Density_map,Sort_P1,ind_P1,GW_Serve_Limit*Range);
    
    [Sort_DP,ind_DP] = sort(Distance_perPixel(Cur_P,:));
    ind_DP = ind_DP(GW_Pathloss_perPixel(Cur_P,ind_DP)>=TxP_Thres);
    Covered2_P = Calculate_Range(Density_map,Sort_DP,ind_DP,GW_Serve_Limit*Range);

    CurP_Covered = (Covered1_P |Covered2_P);

    %% Update Serving range and Covering range of Current point
    CurP_Served1 = zeros(1,size(Rxc,1));
    CurP_Served2 =zeros(1,size(Rxc,1));
    [Sort_P1,ind_p1] = sort(GW_Pathloss_perPixel(Cur_P,:),'descend');
%      CurP_Served1 = Calculate_Range(Density_map,Sort_P1,ind_p1,GW_Serve_Limit);

    [Sort_D,ind_D] = sort(Distance_perPixel(Cur_P,:));
    ind_D = ind_D(find(GW_Pathloss_perPixel(Cur_P,ind_D)>=TxP_Thres));
%     CurP_Served2 = Calculate_Range(Density_map,Sort_D,ind_D,GW_Serve_Limit);
    CurP_Served2 = Calculate_ServingRange(Density_map,Sort_D,ind_D,GW_Serve_Limit,TxP_Thres);

    CurP_Served = (CurP_Served1 | CurP_Served2);
    
    figure;
imshow(floor_plan);
text(Rxc(Cur_P),Rxr(Cur_P),'+','FontSize',20);
for i=1:1:size(Rxr,1)
%     if Tx_ind(i) == 1
%         text(Rxc(i),Rxr(i),'*','Color','Black','FontSize',20);
%     end
    if CurP_Covered(i)==1
        text(Rxc(i),Rxr(i),'o','Color','Black','FontSize',10);
    end
    if CurP_Served(i)==1
        text(Rxc(i),Rxr(i),'x','Color','red','FontSize',10);
    end
%     if Finish_P(i)==1
%         text(Rxc(i),Rxr(i),'x','Color','red','FontSize',10);
%     end
end