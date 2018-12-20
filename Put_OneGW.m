function [lossdB,User_Served,User_Covered,User_Arc] = Put_OneGW(GW_Pathloss_perPixel,CoverRange_perPixel,Tx_ind,Rxc,Rxr,User_Served,User_Arc,User_Covered,lossdB,TxP_Thres,GW_Serve_Limit,Density_map)
Cover_Arc = 180;
Temp_loss =  GW_Pathloss_perPixel(Tx_ind,:);


[Sort_P,ind_P] = sort(Temp_loss(:),'descend');
ServedRange_Thres = max(TxP_Thres,Sort_P(GW_Serve_Limit));
% Served_P = Temp_loss(:) >= ServedRange_Thres;
Served_P = Calculate_Range(Density_map,Sort_P,ind_P,GW_Serve_Limit);
Served_P = find(Served_P == 1);
User_Served(Served_P,1) = Tx_ind;
User_Served(Served_P,2) = Temp_loss(Served_P);

Covered_P1 =find(CoverRange_perPixel(Tx_ind,:)==1);
% Covered_P1 = Covered_P1(find(GW_Pathloss_perPixel(Tx_ind,Covered_P1)>=TxP_Thres));

%% Update lossdB map
for i = 1:1:length(Covered_P1)
    CurP = Covered_P1(i);
    if Temp_loss(CurP) >= TxP_Thres
        if Rxc(Tx_ind)==Rxc(CurP) && Rxr(Tx_ind)==Rxr(CurP)
            User_Arc{CurP,1} = [User_Arc{CurP,1} ,0];
            User_Arc{CurP,2} = [User_Arc{CurP,2} ,359];
        else
            temp_angle = atan2(Rxc(Tx_ind)-Rxc(CurP),Rxr(Tx_ind)-Rxr(CurP))/pi*180;
            if temp_angle < 0
                temp_angle=temp_angle+360;
            end
            temp_angle = mod(temp_angle+270,360);
            %                 text(Rxc(i),Rxr(i),num2str(d),'FontSize',7);
            %                 text(Txc,Txr,'Tx','FontSize',7);
            if (temp_angle - 90) < 0
                User_Arc{CurP,1} = [User_Arc{CurP,1} ,(temp_angle - Cover_Arc/2 + 360),0];
                User_Arc{CurP,2} = [User_Arc{CurP,2} ,359,(temp_angle + Cover_Arc/2)];
                
            elseif (temp_angle + 90) > 359
                User_Arc{CurP,1} = [User_Arc{CurP,1} ,0,(temp_angle - Cover_Arc/2)];
                User_Arc{CurP,2} = [User_Arc{CurP,2} ,(temp_angle - Cover_Arc/2 - 360),359];
                
            else
                User_Arc{CurP,1} = [User_Arc{CurP,1} ,(temp_angle - Cover_Arc/2)];
                User_Arc{CurP,2} = [User_Arc{CurP,2} ,(temp_angle + Cover_Arc/2)];
            end
        end
        User_Covered.GW{CurP} = [User_Covered.GW{CurP} , Tx_ind];
        User_Covered.Num(CurP) = User_Covered.Num(CurP)+1;
        [User_Arc{CurP,3}, User_Arc{CurP,4}] = IntervalUnion(User_Arc{CurP,1},User_Arc{CurP,2});
        low_arc = cell2mat(User_Arc(CurP,3));
        top_arc = cell2mat(User_Arc(CurP,4));
        User_Arc{CurP,5} = 0;
        for r=1:1:length(low_arc)
            User_Arc{CurP,5} = User_Arc{CurP,5} + (top_arc(r) - low_arc(r) + 1);
        end
    end
    %         if Temp_loss(i) > TxP_Thres
    lossdB(CurP) = (Temp_loss(CurP) >= lossdB(CurP))*Temp_loss(CurP)+(Temp_loss(CurP) < lossdB(CurP))*lossdB(CurP);
    %         end
end