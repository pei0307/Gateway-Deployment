function Candidate_P = Find_Cadidate_P_in2GW(P1,P2,GW_Pathloss_perPixel,CoverRange_perPixel,GW_Serve_Limit,TxP_Thres,User_Served)

% GW_Serve_Limit = (6-1)*3/0.25;
[Sort_P1,~] = sort(GW_Pathloss_perPixel(P1,:),'descend');
ServedRange_Thres = max(TxP_Thres,Sort_P1(GW_Serve_Limit));
Served_P1 = find(GW_Pathloss_perPixel(P1,:) >= ServedRange_Thres);

Covered_P1 = find(CoverRange_perPixel(P1,:)==1);
% Covered_P1 =  find(GW_Pathloss_perPixel(P1,:) >= TxP_Thres);


Sort_P2 = sort(GW_Pathloss_perPixel(P2,:),'descend');
ServedRange_Thres = max(TxP_Thres,Sort_P2(GW_Serve_Limit));
Served_P2 = find(GW_Pathloss_perPixel(P2,:) >= ServedRange_Thres);

Covered_P2 = find(CoverRange_perPixel(P2,:)==1);
% Covered_P2 =  find(GW_Pathloss_perPixel(P2,:) >= TxP_Thres);

Candidate_P = intersect(Covered_P1,Covered_P2);
% Candidate_P = intersect(Served_P1,Served_P2);

end
