function Covered_P = Calculate_Range(Density_map,SortDP,ind_DP,GW_Serve_Limit)
Covered_P = zeros(1,length(Density_map));
Capacity = 0;
Power = 0;
for i=1:1:length(ind_DP)
    if Capacity < GW_Serve_Limit || SortDP(i) == Power
        Capacity = Capacity + Density_map(ind_DP(i));
        Covered_P(ind_DP(i)) = 1;
        Power = SortDP(i);
    end
end
end