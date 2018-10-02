function [lossdB,User_Served,User_Covered,User_Arc] = Deploy_Result(GW_Pathloss_perPixel,CoverRange_perPixel,Tx_ind,Rxr,Rxc,TxP_Thres,GW_Serve_Limit)

Cover_Arc = 180;

User_Served = zeros(size(Rxr,1),2);   %(GW_ID,Txpower)
User_Covered.GW = cell(size(Rxr,1),1);
User_Covered.Num = zeros(size(Rxr,1),1);
User_Arc = cell(size(Rxr,1),2);
Range = 4;

GW_Pathloss = (GW_Pathloss_perPixel).*(Tx_ind')+(-300).*(~Tx_ind');
temp_Tx = find(Tx_ind==1);
for i=1:1:length(temp_Tx)
    Distance_P = find(CoverRange_perPixel(temp_Tx(i),:)==1);

    for j=1:1:length(Distance_P)
        User_Covered.GW{Distance_P(j)} = [User_Covered.GW{Distance_P(j)}, temp_Tx(i)];
        User_Covered.Num(Distance_P(j)) = User_Covered.Num(Distance_P(j))+1;
    end
    
end
for i=1:1:size(Rxr,1)
    [cP,ind]= sort(GW_Pathloss_perPixel(i,cell2mat(User_Covered.GW(i))),'descend');
    if ~isempty(cP)
        User_Served(i,1) = User_Covered.GW{i}(ind(1));
        User_Served(i,2) = cP(1);
    end
%     [User_Served(i,2),User_Served(i,1)] = max(GW_Pathloss(:,i));

    lossdB(i) = max(GW_Pathloss(:,i));
    
    cGW = User_Covered.GW{i}(ind);
    User_Arc{i,6} = cP;
    for j=1:1:User_Covered.Num(i)
        if Rxc(cGW(j))==Rxc(i) && Rxr(cGW(j))==Rxr(i)
            User_Arc{i,1} = [User_Arc{i,1} ,0];
            User_Arc{i,2} = [User_Arc{i,2} ,359];
        else
            temp_angle = atan2(Rxc(cGW(j))-Rxc(i),Rxr(cGW(j))-Rxr(i))/pi*180;
            if temp_angle < 0
                temp_angle=temp_angle+360;
            end
            temp_angle = mod(temp_angle+270,360);
            %                 text(Rxc(i),Rxr(i),num2str(d),'FontSize',7);
            %                 text(Txc,Txr,'Tx','FontSize',7);
            if (temp_angle - 90) < 0
                User_Arc{i,1} = [User_Arc{i,1} ,(temp_angle - Cover_Arc/2 + 360),0];
                User_Arc{i,2} = [User_Arc{i,2} ,359,(temp_angle + Cover_Arc/2)];
          
            elseif (temp_angle + 90) > 359
                User_Arc{i,1} = [User_Arc{i,1} ,0,(temp_angle - Cover_Arc/2)];
                User_Arc{i,2} = [User_Arc{i,2} ,(temp_angle + Cover_Arc/2 - 360),359];
                
            else
                User_Arc{i,1} = [User_Arc{i,1} ,(temp_angle - Cover_Arc/2),0];
                User_Arc{i,2} = [User_Arc{i,2} ,(temp_angle + Cover_Arc/2),0];
            end
            
        end
        [User_Arc{i,3}, User_Arc{i,4}] = IntervalUnion(User_Arc{i,1},User_Arc{i,2});
        low_arc = cell2mat(User_Arc(i,3));
        top_arc = cell2mat(User_Arc(i,4));
        User_Arc{i,5} = 0;
        for r=1:1:length(low_arc)
            User_Arc{i,5} = User_Arc{i,5} + (top_arc(r) - low_arc(r) + 1);
        end
    end
end
temp_Tx = find(Tx_ind==1);
Tx_Record = zeros(1,size(Rxc,1));
for i=1:1:length(temp_Tx)
    Covered_P = find(CoverRange_perPixel(temp_Tx(i),:)==1);
    [Served_P,ind]= sort(GW_Pathloss_perPixel(temp_Tx(i),Covered_P),'descend');
    attach = 0;
    for j =1:1:length(Served_P)
        User = Covered_P(ind(j));
        %if user has not been served or served power lower than this GW
        if  attach < GW_Serve_Limit && User_Served(User,1) == temp_Tx(i) 
            attach =attach + 1;
        elseif (attach == GW_Serve_Limit && User_Served(User,1) == temp_Tx(i))
            User_Served(User,1) = 0;
            User_Served(User,2) = 0;
        end
    end
    Tx_Record(temp_Tx(i)) = attach;
end

for i=1:1:length(temp_Tx)
    if  Tx_Record(temp_Tx(i)) < GW_Serve_Limit
        Covered_P = find(CoverRange_perPixel(temp_Tx(i),:)==1);
        [Served_P,ind]= sort(GW_Pathloss_perPixel(temp_Tx(i),Covered_P),'descend');
        for j =1:1:length(ind)
            User = Covered_P(ind(j));
            if User_Served(User,1) == 0
                User_Served(User,1) = temp_Tx(i);
                User_Served(User,2) = Served_P(j);
                Tx_Record(temp_Tx(i)) = Tx_Record(temp_Tx(i))+1;
                if Tx_Record(temp_Tx(i)) == GW_Serve_Limit
                    break;
                end
            end
        end
    end
end

