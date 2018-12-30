function [User,Tx_Record,Fail_ind] = GW_Assignment(User,Rxc,Rxr,User_num,User_Covered,User_Arc,GW_Serve_Limit,Fail_ind)
User(:,3) = 0;
User(:,4) = 0;
Tx_Record = zeros(1,size(Rxc,1));
for i=1:1:User_num
    for j = 1:1:User_Covered.Num(User(i))

        temp_angle = atan2(Rxc(User_Covered.GW{User(i)}(j))-Rxc(User(i)),Rxr(User_Covered.GW{User(i)}(j))-Rxr(User(i)))/pi*180;
        if temp_angle < 0
            temp_angle=temp_angle+360;
        end
        if (User(i,1) ==User_Covered.GW{User(i)}(j) || abs(User(i,2) -temp_angle) <=90) && Tx_Record( User_Covered.GW{User(i)}(j)) < GW_Serve_Limit
            User(i,3) = User_Covered.GW{User(i)}(j);
            User(i,4) = User_Arc{User(i),6}(j);
            Tx_Record( User_Covered.GW{User(i)}(j)) = Tx_Record( User_Covered.GW{User(i)}(j))+1;
            break;
        end
    end
    if User(i,3) == 0
        Fail_ind(User(i,1)) = Fail_ind(User(i,1))+1;
    end
end
end