function User  = Random_walk(User,User_num,meshNode)

for i=1:1:User_num
    while(1)
        angle = randi([0,8]);
        user_id =  User(i,1);
        if angle == 0 && user_id<=meshNode.vert.num*(meshNode.horz.num-1)
            User(i,1) = User(i,1)+meshNode.vert.num;
            User(i,2) = angle*45;
            break;
        elseif  angle == 1 && user_id<=meshNode.vert.num*(meshNode.horz.num-1) && mod(user_id,meshNode.vert.num)~=1
            User(i,1) = User(i,1)+meshNode.vert.num-1;
            User(i,2) = angle*45;
            break;
        elseif angle == 2 && mod(user_id,meshNode.vert.num)~=1
            User(i,1) = User(i,1)-1;
            User(i,2) = angle*45;
            break;
        elseif angle == 3 && user_id > meshNode.vert.num && mod(user_id,meshNode.vert.num)~=1
            User(i,1) = User(i,1)-meshNode.vert.num-1;
            User(i,2) = angle*45;
            break;
        elseif angle == 4 && user_id > meshNode.vert.num
            User(i,1) = User(i,1)-meshNode.vert.num;
            User(i,2) = angle*45;
            break;
        elseif angle == 5  && user_id > meshNode.vert.num && mod(user_id,meshNode.vert.num)~=0
            User(i,1) = User(i,1)-meshNode.vert.num+1;
            User(i,2) = angle*45;
            break;
        elseif angle == 6 && mod(user_id,meshNode.vert.num)~=0
            User(i,1) = User(i,1)+1;
            User(i,2) = angle*45;
            break;
        elseif angle == 7 && user_id<=meshNode.vert.num*(meshNode.horz.num-1) && mod(user_id,meshNode.vert.num)~=0
            User(i,1) = User(i,1)+meshNode.vert.num+1;
            User(i,2) = angle*45;
            break;
        else
            break;
        end
    end
end
end