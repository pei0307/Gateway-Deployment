function Served_P = Calculate_ServingRange(Density_map,SortDP,ind_DP,GW_Serve_Limit,TxP_Thres)
Served_P = zeros(1,length(Density_map));
Capacity = 0;
for i=1:1:length(ind_DP)
    if Capacity < GW_Serve_Limit
        Capacity = Capacity + Density_map(ind_DP(i));
        if SortDP(i)>=TxP_Thres
            Served_P(ind_DP(i)) = 1;
        end
    end
end
end