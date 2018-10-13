function Fulfill = Constraint_Fulfilled(Cur_Served,User_Served,User_Arc)
% Association constraint(1)
F1 = length(find(User_Served(Cur_Served)>0))/length(Cur_Served);

% Full Covered Guarantee(5)
F5_Satisfy = 0;
for i = 1:1:length(Cur_Served)
    [User_Arc{Cur_Served(i),3}, User_Arc{Cur_Served(i),4}] = IntervalUnion(User_Arc{Cur_Served(i),1},User_Arc{Cur_Served(i),2});
    low_arc = cell2mat(User_Arc(Cur_Served(i),3));
    top_arc = cell2mat(User_Arc(Cur_Served(i),4));
    User_Arc{Cur_Served(i),5} = 0;
    for r=1:1:length(low_arc)
        User_Arc{Cur_Served(i),5} = User_Arc{Cur_Served(i),5} + (top_arc(r) - low_arc(r) + 1);
    end
    if User_Arc{Cur_Served(i),5} >= 360
        F5_Satisfy = F5_Satisfy + 1;
    elseif ~isempty(User_Arc{Cur_Served(i),5})
        F5_Satisfy = F5_Satisfy + User_Arc{Cur_Served(i),5} / 360;
    end
end
F5 = F5_Satisfy/length(Cur_Served);

Fulfill = (F1+F5)/2;
end
