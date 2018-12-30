function [lossdB,User_Served,User_Covered,User_Arc] = Deploy_Result3(GW_Pathloss_perPixel,Tx_ind,Rxr,Rxc,TxP_Thres,GW_Serve_Limit)

Cover_Arc = 180;

User_Served = zeros(size(Rxr,1),2);   %(GW_ID,Txpower)
User_Covered.GW = cell(size(Rxr,1),1);
User_Covered.Num = zeros(size(Rxr,1),1);
User_Arc = cell(size(Rxr,1),2);

GW_Pathloss = (GW_Pathloss_perPixel).*(Tx_ind')+(-300).*(~Tx_ind');
for i=1:1:size(Rxr,1)
    [User_Served(i,2),User_Served(i,1)] = max(GW_Pathloss(:,i));
    lossdB(i) = User_Served(i,2);
    cGW_t = find(GW_Pathloss(:,i) >= TxP_Thres)';
    cP = GW_Pathloss(cGW_t,i);
    [Psort,pixel_id] = sort(cP,'descend');
    cGW = cGW_t(pixel_id);
    User_Covered.GW{i} = cGW;
    User_Covered.Num(i) = length(cGW);
    for j=1:1:User_Covered.Num(i)
        if Rxc(cGW(j))==Rxc(i) && Rxr(cGW(j))==Rxr(i)
            User_Arc{i,1} = [User_Arc{i,1} ,0,0];
            User_Arc{i,2} = [User_Arc{i,2} ,359,0];
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
        User_Arc{i,6} = Psort;
        [User_Arc{i,3}, User_Arc{i,4}] = IntervalUnion(User_Arc{i,1},User_Arc{i,2});
        low_arc = cell2mat(User_Arc(i,3));
        top_arc = cell2mat(User_Arc(i,4));
        User_Arc{i,5} = 0;
        for r=1:1:length(low_arc)
            User_Arc{i,5} = User_Arc{i,5} + (top_arc(r) - low_arc(r) + 1);
        end
        User_Arc{i,1}=User_Arc{i,3};
        User_Arc{i,2}=User_Arc{i,4};
     
    end
end
temp_Tx = find(Tx_ind==1);
Tx_Record = zeros(1,size(Rxc,1));
for i=1:1:length(temp_Tx)
    [Served_P,ind] = sort(GW_Pathloss_perPixel(temp_Tx(i),:),'descend');
%     Served_P = find(GW_Pathloss_perPixel(temp_Tx(i),:) >= Sort_Power(GW_Serve_Limit));
%     [value,ind] = sort(GW_Pathloss_perPixel(temp_Tx(i),Served_P),'descend');
    attach = 0;
    for j =1:1:length(Served_P)
        %if user has not been served or served power lower than this GW
        if  attach < GW_Serve_Limit && User_Served(ind(j),1) == temp_Tx(i)  
            attach =attach + 1;
        elseif attach == GW_Serve_Limit && User_Served(ind(j),1) == temp_Tx(i)
            User_Served(ind(j),1) = 0;
            User_Served(ind(j),2) = 0;
        end
    end
    Tx_Record(temp_Tx(i)) = attach;
end

for i=1:1:length(temp_Tx)
   if  Tx_Record(temp_Tx(i)) < GW_Serve_Limit
       [Served_P,ind] = sort(GW_Pathloss_perPixel(temp_Tx(i),:),'descend');
       for j =1:1:length(ind)
            if User_Served(ind(j),1) == 0
                if Served_P(j) >= TxP_Thres
                    User_Served(ind(j),1) = temp_Tx(i);
                    User_Served(ind(j),2) = Served_P(j);
                    Tx_Record(temp_Tx(i)) = Tx_Record(temp_Tx(i))+1;
                    if Tx_Record(temp_Tx(i)) == GW_Serve_Limit
                        break;
                    end
                end
            end
       end
   end
end

