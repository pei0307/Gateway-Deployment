function User  = Random_walk(User,User_num,meshNode,Distance_perPixel,isWall_perPixel)

for i=1:1:User_num
    while(1)
        angle = randi([0,8]);
        user_id =  User(i,1);
        [Dis,ind] = sort(Distance_perPixel(user_id,:));
        pixel = Dis(2);
        if angle ~= 0
            if Dis(angle)<=pixel+1 && ind(angle) == user_id-1 %上
                User(i,1) =  ind(angle);
                User(i,2) = (angle-1)*45;
                break;
            elseif Dis(angle)<=pixel+1 && ind(angle) == user_id+1 %下
                User(i,1) =  ind(angle);
                User(i,2) = (angle-1)*45;
                break;
            elseif Dis(angle) <= pixel+1 && ind(angle) < user_id %左
                User(i,1) =  ind(angle);
                User(i,2) = (angle-1)*45;
                break;
            elseif Dis(angle) <= pixel+1 && ind(angle) > user_id %右
                User(i,1) =  ind(angle);
                User(i,2) = (angle-1)*45;
                break;
            elseif Dis(angle) < 2*pixel && ind(angle) < user_id && ind(angle) ==  min(ind) %左上
                User(i,1) =  ind(angle);
                User(i,2) = (angle-1)*45;
                break;
            elseif Dis(angle) < 2*pixel && ind(angle) < user_id %左下
                User(i,1) =  ind(angle);
                User(i,2) = (angle-1)*45;
                break;
            elseif Dis(angle) < 2*pixel && ind(angle) > user_id && ind(angle) ==  max(ind) %右下
                User(i,1) =  ind(angle);
                User(i,2) = (angle-1)*45;
                break;
            elseif Dis(angle) < 2*pixel && ind(angle) > user_id %右上
                User(i,1) =  ind(angle);
                User(i,2) = (angle-1)*45;
                break;
            end
        else
            break;
        end
    end
end
end