function Fitness = Calculate_Fitness(Cur_Served,User_Arc)
% Full Covered Guarantee(5)
F5_Satisfy = 0;
for i = 1:1:length(Cur_Served)
    if User_Arc{Cur_Served(i),5} >= 360
        F5_Satisfy = F5_Satisfy + 1;
    elseif ~isempty(User_Arc{Cur_Served(i),5})
        F5_Satisfy = F5_Satisfy + User_Arc{Cur_Served(i),5} / 360;
    end
end
F5 = F5_Satisfy/length(Cur_Served);
Fitness = F5;
end
