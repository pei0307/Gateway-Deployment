function User  = Random_walk(User,User_num,meshNode)

for i=1:1:User_num
    while(1)
        angle = randi([0,8]);
        user_id =  User(i,1);
        if User(i,1) <= size(User,1)
            if angle == 0 && User(i,1)+meshNode.vert.num<=size(User,1) &&User(i,1)+meshNode.vert.num>0
                User(i,1) = User(i,1)+meshNode.vert.num;
                User(i,2) = angle*45;
                break;
            elseif  angle == 1 && User(i,1)+meshNode.vert.num-1<=size(User,1)&&User(i,1)+meshNode.vert.num-1>0
                User(i,1) = User(i,1)+meshNode.vert.num-1;
                User(i,2) = angle*45;
                break;
            elseif angle == 2 && User(i,1)-1 <=size(User,1)&&User(i,1)-1>0
                User(i,1) = User(i,1)-1;
                User(i,2) = angle*45;
                break;
            elseif angle == 3 && User(i,1)-meshNode.vert.num-1 <= size(User,1)&&User(i,1)-meshNode.vert.num-1>0
                User(i,1) = User(i,1)-meshNode.vert.num-1;
                User(i,2) = angle*45;
                break;
            elseif angle == 4 && User(i,1)-meshNode.vert.num <= size(User,1) && User(i,1)-meshNode.vert.num>0
                User(i,1) = User(i,1)-meshNode.vert.num;
                User(i,2) = angle*45;
                break;
            elseif angle == 5  && User(i,1)-meshNode.vert.num+1 <= size(User,1) && User(i,1)-meshNode.vert.num+1>0
                User(i,1) = User(i,1)-meshNode.vert.num+1;
                User(i,2) = angle*45;
                break;
            elseif angle == 6 &&  User(i,1)+1 <= size(User,1)
                User(i,1) = User(i,1)+1;
                User(i,2) = angle*45;
                break;
            elseif angle == 7 && User(i,1)+meshNode.vert.num+1 <=size(User,1) && User(i,1)+meshNode.vert.num+1>0
                User(i,1) = User(i,1)+meshNode.vert.num+1;
                User(i,2) = angle*45;
                break;
            elseif angle == 8
                break;
            end
        end
    end
end
end