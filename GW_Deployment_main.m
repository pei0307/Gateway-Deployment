clear;

%% Wall Detection Parameters (Change them wiselt if walls are not correctly detected)
thetaRes = 0.1;                   % Resolution of the Hough Transform Space (don't make it smaller than 0.1)
minWallLength = 5;               % Minimum length of the walls in pixel
fillGap = 3;                     % Gap between walls
%Wall Attenuation Coefficient Assignment dB
% Manually Assign Attenuation factors to each particular wall based on it's
% intensity dynamic range (change plot mode in autoWallDetection to 1 to
% see the intesity of each wall.
wallAt = zeros(255,1);
wallAt(1:end) = 10;
wallAt(255) = round(sum(wallAt)./sum(wallAt>0));  % This is for intersecting walls, just leave it as it is

%% Pixel Setting Parameter
calibrate_option        = 0;
Pixel_Res = 1;    %pixel=2m*2m
pathLossModel           = 2;        % 1=Free-Space, 2=our pathloss
demoMode                = 0;        % Showes furtther details if 1

%% Pathloss Setting Parameter
TxPower                 = -63.8;        % dBm or dB
BER2SNR = @(BER)((erfcinv(2*BER))^2)*2;
SNR_Thres = BER2SNR(10^(-3));
noise = -94; %dbm
TxP_Thres = SNR_Thres + noise ;
Data_Rate = (2*(256)*8)/1000; % kbps
Max_throughput = 113; %kbps(BER=10^-3)
Control_Period = 294.7580; %ms
Density = 0.5;
GW_Serve_Limit = floor((Max_throughput*(1000-Control_Period)/1000)/Data_Rate/3)/Density;
PdBm_to_Pw = @(PdBm) 10^(PdBm/10);
Pw_to_PdBm = @(Pw) 10*log10(Pw);



floor_plan = 'data\thesis.png';
wall_detect = [floor_plan(1:length(floor_plan)-4),'_wall_detect.mat'];
Pixel_Setting = [floor_plan(1:length(floor_plan)-4),'_pixel_Setting.mat'];
Pathloss_Distance = [floor_plan(1:length(floor_plan)-4),'_PlandDis_perpixel.mat'];

% load floor plan
floorPlan = imread(floor_plan);
floorPlanBW = im2bw(floorPlan);
originalFloorPlan = floorPlanBW;

% wall detection
if exist(wall_detect, 'file')
    load (wall_detect) ;
else
    [floorPlanGray,countedWalls] = autoWallDetection(~originalFloorPlan,wallAt,0,thetaRes,minWallLength,fillGap); % Detecting all the walls Generates floorPlanGray where different wall are index coded in the gray image.
    save(wall_detect,'floorPlanGray','countedWalls');
end

% Pixel Setting
if exist(Pixel_Setting, 'file')
    load (Pixel_Setting);
else
    % Getting 2 points from image for calibration
    floorPlanBW = ~imdilate(~floorPlanBW,strel('disk',3));
    if calibrate_option == 0
        [R,C,floorPlanBW] = Calibration_Manual(floorPlanBW);
    elseif calibrate_option == 1
        R(1,1) = 1;
        R(2,1) = 298;
        C(1,1) = 1;
        C(2,1) = 1;
    end
    % Finding shortest path betweent the two points
    calibPath = shortestPath(imcomplement(floorPlanBW),R,C);
    P = imoverlay(floorPlanBW,calibPath,[0,1,0]);
    
    % Calibrating pixel per meter
    pathPixels = sum(sum(calibPath));
    pathLength = input('Length of calibration path in meters: ');
    pathUnit = pathLength./pathPixels; %pathUnit meter per pixel
    
    
    % Number of probs in the structure increase for better accuracy
    meshNode.vert.num       = round(size(floorPlanBW,1)/(Pixel_Res/pathUnit));
    meshNode.horz.num       = round(size(floorPlanBW,2)/(Pixel_Res/pathUnit));
    Unit = Pixel_Res/pathUnit;
    
    mesh.vert.spacing = pathUnit .* size(floorPlanBW,1) ./ meshNode.vert.num; % node spacing meters
    mesh.horz.spacing = pathUnit .* size(floorPlanBW,2) ./ meshNode.horz.num; % node spacing meters
    
    % Meshing the Floor Plan
    floorMesh = zeros(size(floorPlanBW));
    for i=1:1: meshNode.horz.num
        if i*Unit  < size(floorPlanBW,2)
            horz =round(i*Unit - Unit/2);
        else
            %             horz = size(floorPlanBW,2)-1;
        end
        for j=1:1:meshNode.vert.num
            
            if j*Unit  < size(floorPlanBW,1)
                vert = round(j*Unit - Unit/2);
            else
                %             vert = size(floorPlanBW,1)-1;
            end
            floorMesh(vert,horz)=1;
        end
    end
    
    % finding the nodes
    [Rxr,Rxc] = find(floorMesh == 1);
    for i=1:1:size(Rxr,1)
        if floorPlanGray(Rxr(i),Rxc(i))~=0
            while(1)
                t_Rxc = Rxc(i) + round(rand(1)*2-1);
                t_Rxr = Rxr(i) + round(rand(1)*2-1);
                if(floorPlanGray(t_Rxr,t_Rxc) == 0)
                    Rxr(i) = t_Rxr;
                    Rxc(i) = t_Rxc;
                    break;
                end
            end
        end
    end
    save(Pixel_Setting,'pathUnit','meshNode','Rxc','Rxr');
end

if exist(Pathloss_Distance,'file')
    load(Pathloss_Distance);
else
    %% Calculate_Pathloss
    Tx_ind = ones(1,size(Rxc,1));
    GW_Pathloss_perPixel = Calculate_Pathloss(originalFloorPlan,floorPlanGray,wallAt,pathLossModel,Tx_ind,Rxc,Rxr,pathUnit,TxP_Thres);
    %% Calculate_Distance
    Distance_perPixel = zeros(size(Rxc,1),size(Rxc,1));
    for i=1:1:size(Rxc,1)
        for j=1:1:size(Rxc,1)
            Distance_perPixel(i,j) = sqrt((Rxc(i)-Rxc(j))^2+(Rxr(i)-Rxr(j))^2);
        end
    end
    save(Pathloss_Distance,'GW_Pathloss_perPixel','Distance_perPixel');
end

%% Initialization
initial_Tx = [1,meshNode.vert.num,meshNode.vert.num*((meshNode.horz.num)-1)+1,meshNode.vert.num*(meshNode.horz.num)];
Tx_ind = zeros(1,size(Rxc,1));
Tx_Record = zeros(1,size(Rxc,1));
Queue_ind = zeros(1,size(Rxc,1));
Tx_ind(initial_Tx) =1;
q_num = length(find(Tx_ind==1));
GW_Num = length(find(Tx_ind==1));
Queue_ind(initial_Tx) = [1:1:GW_Num];
Finish_P = zeros(1,size(Rxc,1));
Range = 4;

%% Calculate Covering range
CoverRange_perPixel = zeros(size(Rxc,1),size(Rxc,1));
for i=1:1:size(Rxc,1)
    Covered1_P = zeros(1,size(Rxc,1));
    Covered2_P =zeros(1,size(Rxc,1));
    Covered1_P = GW_Pathloss_perPixel(i,:) >=TxP_Thres;
    [Sort_P1,ind_P1] = sort(GW_Pathloss_perPixel(i,:),'descend');
    Covered1_P = (GW_Pathloss_perPixel(i,:) >= Sort_P1(GW_Serve_Limit*Range));
    [Sort_DP,ind_DP] = sort(Distance_perPixel(i,:));
    ind_DP = ind_DP(find(GW_Pathloss_perPixel(i,ind_DP)>=TxP_Thres));
    ind_DP = ind_DP(1:min(GW_Serve_Limit*Range,length(ind_DP)));
    for j=1:1:length(ind_DP)
        Covered2_P(ind_DP(j)) =1;
    end
    Covered_P = (Covered1_P |Covered2_P);
    CoverRange_perPixel(i,:) = Covered_P;
end
for i=1:1:size(Rxc,1)
    CoverRange_perPixel(i,:) =((CoverRange_perPixel(i,:) ) | (CoverRange_perPixel(:,i)'));
end

while (1)
    %% Dequeue()
    [t_value,t_ind] = min((Queue_ind + (Queue_ind<1)*size(Rxc,1)));
    if length(find(Queue_ind>0))>0
        Cur_P = t_ind;
    else
        break;
    end
    fprintf(' Cur_P = %d\n',Cur_P);
    %% Update Serving range and Covering range of Current point
    CurP_Served1 = zeros(1,size(Rxc,1));
    CurP_Served2 =zeros(1,size(Rxc,1));
    [Sort_P1,ind_p1] = sort(GW_Pathloss_perPixel(Cur_P,:),'descend');
    ServedRange_Thres = max(TxP_Thres,Sort_P1(GW_Serve_Limit));
    %  CurP_Served1 = GW_Pathloss_perPixel(Cur_P,:) >= ServedRange_Thres;
    [Sort_D,ind_D] = sort(Distance_perPixel(Cur_P,:));
    ind_D = ind_D(find(GW_Pathloss_perPixel(Cur_P,ind_D)>=TxP_Thres));
    ind_D = ind_D(1:min(GW_Serve_Limit,length(ind_D)));
    for j=1:1:length(ind_D)
        CurP_Served2(ind_D(j)) =1;
    end
    CurP_Served = (CurP_Served1 | CurP_Served2);
    CurP_Covered = find(CoverRange_perPixel(Cur_P,:)==1);
     
    
    [lossdB,User_Served,User_Covered,User_Arc] = Deploy_Result(GW_Pathloss_perPixel,CoverRange_perPixel,Tx_ind,Rxr,Rxc,TxP_Thres,GW_Serve_Limit);
    %% Construct the triangle and Update the Finish Set
    Link = intersect(find(Tx_ind==1),CurP_Covered);
    for k=1:1:length(Link)
        Candidate_P = Find_Cadidate_P_in2GW(Cur_P,Link(k),GW_Pathloss_perPixel,CoverRange_perPixel,GW_Serve_Limit,TxP_Thres,User_Served);
        Candidate_P = intersect(Candidate_P,find(Tx_ind==1));
        for j=1:1:length(Candidate_P)
            % find the point inside the triangle
            if Cur_P~=Candidate_P(j) && Link(k)~=Candidate_P(j) && Cur_P~= Link(k)
                P1 = [Rxc(Cur_P),Rxr(Cur_P)];
                P2 = [Rxc(Link(k)),Rxr(Link(k))];
                P3 = [Rxc(Candidate_P(j)),Rxr(Candidate_P(j))];
                InsideTri_P  = zeros(1,size(Rxc,1));
                s = det([P1-P2;P3-P1]);
                for r= 1:1:size(Rxc,1)
                    P = [Rxc(r),Rxr(r)];
                    InsideTri_P(r) = (InsideTri_P(r)|(s*det([P3-P;P2-P3])>=0 & s*det([P1-P;P3-P1])>=0 & s*det([P2-P;P1-P2])>=0));
                end
                Tri_Utility = Constraint_Fulfilled(find(InsideTri_P==1),User_Served,User_Arc);
                if Tri_Utility == 1 && length(find(InsideTri_P.*(~Finish_P)))>0
                    fprintf('FInish::::CurP = %d,Link = %d,Candidate = %d\n',Cur_P,Link(k),Candidate_P(j));
                    Finish_P = or(Finish_P , InsideTri_P);
                    % draw the triangle
                    x= [Rxc(Cur_P),Rxc(Candidate_P(j)),Rxc(Link(k)),Rxc(Cur_P)];
                    y= [Rxr(Cur_P),Rxr(Candidate_P(j)),Rxr(Link(k)),Rxr(Cur_P)];
                    plot(x,y);
                end
            end
        end
    end
     
    %% Is GW_cur¡¦s serving range (S_cur) fulfilled the constraint?
    CurP_Utility = Constraint_Fulfilled(find(CurP_Served==1),User_Served,User_Arc);
    if CurP_Utility < 1 || length(find((CurP_Served.*(Finish_P|Tx_ind))==1)) < length(find((CurP_Served==1)))
        % find the GW  from the covering range as the candidate link
        candidate_link = find(Queue_ind > t_value);
        Link = intersect(candidate_link , CurP_Covered);
        if ~isempty(Link)
            while(~isempty(Link))
                fprintf('Link length= %d   %s \n',length(Link),num2str(Link));
                Best_Fitness = 0;
                Cur_Finish = 0;
                % Construct the triangle and find the highest fitness one
                for k=1:1:length(Link)
                    Candidate_P = Find_Cadidate_P_in2GW(Cur_P,Link(k),GW_Pathloss_perPixel,CoverRange_perPixel,GW_Serve_Limit,TxP_Thres,User_Served);
                    Temp_Fitness = 0;
                    for j=1:1:length(Candidate_P)
                        P1 = [Rxc(Cur_P),Rxr(Cur_P)];
                        P2 = [Rxc(Link(k)),Rxr(Link(k))];
                        P3 = [Rxc(Candidate_P(j)),Rxr(Candidate_P(j))];
                        if sum((P1-P2)/sum((P1-P2)) ~= (P2-P3)/sum((P2-P3)))==2 && Finish_P(Candidate_P(j)) ~= 1 && Tx_ind(Candidate_P(j)) ~=1
                            temp_Tx_ind = Tx_ind;
                            temp_Tx_ind(Candidate_P(j)) =1;
                            InsideTri_P  = zeros(1,size(Rxc,1));
                            s = det([P1-P2;P3-P1]);
                            for r= 1:1:size(Rxc,1)
                                P = [Rxc(r),Rxr(r)];
                                InsideTri_P(r) = (InsideTri_P(r)|(s*det([P3-P;P2-P3])>=0 & s*det([P1-P;P3-P1])>=0 & s*det([P2-P;P1-P2])>=0));
                            end
                            [~,try_User_Served,try_User_Covered,try_User_Arc] = Put_OneGW(GW_Pathloss_perPixel,CoverRange_perPixel,Candidate_P(j),Rxc,Rxr,User_Served,User_Arc,User_Covered,lossdB,TxP_Thres,GW_Serve_Limit,temp_Tx_ind);
                            % the fitness of all the pixels
                            Fitness = Calculate_Fitness([1:1:size(Rxc,1)],try_User_Arc);
                            Cur_Utility = Constraint_Fulfilled(find(CurP_Served==1),try_User_Served,try_User_Arc);
                            Tri_Utility = Constraint_Fulfilled(find(InsideTri_P==1),try_User_Served,try_User_Arc);
                            Fitness =  (Fitness)*(length(find((InsideTri_P.*CurP_Served)==1))-length(find((InsideTri_P.*Finish_P.*CurP_Served)==1)));
                            % best utility or current point 's served range is finished
                            if ((Fitness > Temp_Fitness) || Cur_Utility >= 1 )&& Tri_Utility==1
                                if Cur_Utility >=1
                                    if Cur_Finish == 0
                                        Temp_Fitness = Fitness;
                                        Temp_P = Candidate_P(j);
                                        Temp_Tri_P = InsideTri_P;
                                        Cur_Finish = 1;
                                    elseif (Fitness > Temp_Fitness)
                                        Temp_Fitness = Fitness;
                                        Temp_P = Candidate_P(j);
                                        Temp_Tri_P = InsideTri_P;
                                    end
                                elseif Cur_Finish == 0
                                    Temp_Fitness = Fitness;
                                    Temp_P = Candidate_P(j);
                                    Temp_Tri_P = InsideTri_P;
                                end
                            end
                        end
                    end
                    if Temp_Fitness ~= 0 && Temp_Fitness > Best_Fitness
                        Best_Link = Link(k);
                        Best_Fitness = Temp_Fitness;
                        Best_P = Temp_P;
                        Best_Tri_P = Temp_Tri_P;
                    end
                end
                
                if Best_Fitness ~= 0
                    %% Put a new GW and Enqueue()
                    Tx_ind(Best_P) = 1;
                    Finish_P = or(Finish_P , Best_Tri_P);
                    fprintf('Cur_P = %d , Best_Link = %d , Best_P = %d ,Utility = %d\n',Cur_P,Best_Link,Best_P,Best_Fitness);
                    x= [Rxc(Cur_P),Rxc(Best_Link),Rxc(Best_P),Rxc(Cur_P)];
                    y= [Rxr(Cur_P),Rxr(Best_Link),Rxr(Best_P),Rxr(Cur_P)];
                    plot(x,y);
                    q_num = q_num+1;
                    Queue_ind(Best_P) = q_num;
                    Link = [Link,Best_P];
                else
                    fprintf('Best_Utility = 0\n');
                    break;
                end
                if Cur_Finish ==1
                    Queue_ind(Cur_P) = -1;
                    fprintf('Current point Serving range are fininshed.\n');
                    break;
                end
                [lossdB,User_Served,User_Covered,User_Arc] = Deploy_Result(GW_Pathloss_perPixel,CoverRange_perPixel,Tx_ind,Rxr,Rxc,TxP_Thres,GW_Serve_Limit);
                Cur_Utility = Constraint_Fulfilled(find(CurP_Served==1),User_Served,User_Arc);
                if Cur_Utility >=1
                    break;
                end
            end
        end
        if (length(Link) == 0 || Best_Fitness == 0)
            Candidate_P =  find(CoverRange_perPixel(Cur_P,:)==1);
            Best_Fitness = 0;
            for j=1:1:length(Candidate_P)
                if Finish_P(Candidate_P(j)) ~= 1 && Tx_ind(Candidate_P(j))~=1
                    temp_Tx_ind = Tx_ind;
                    temp_Tx_ind(Candidate_P(j)) =1;
                    [try_lossdB,try_User_Served,try_User_Covered,try_User_Arc] = ...
                        Deploy_Result(GW_Pathloss_perPixel,CoverRange_perPixel,temp_Tx_ind,Rxr,Rxc,TxP_Thres,GW_Serve_Limit);
                    Fitness = Calculate_Fitness([1:1:size(Rxc,1)],try_User_Arc);
                    Cur_Utility = Constraint_Fulfilled(find(CurP_Served==1),try_User_Served,try_User_Arc);
                    if (Fitness > Best_Fitness)
                        Best_P = Candidate_P(j);
                        Best_Fitness = Fitness;
                    end
                end
            end
            if Best_Fitness ~=0
                %% Put a new GW and Enqueue()
                Tx_ind(Best_P) =1;
                q_num = q_num+1;
                Queue_ind(Best_P) = q_num;
                fprintf('Cur_P = %d , Best_P = %d \n',Cur_P,Best_P);
            else
                Finish_P = or(Finish_P , CurP_Served);
                Queue_ind(Cur_P) = -1;
                fprintf('Cannot find the link then finish current point\n');
            end
        end
    else
        Queue_ind(Cur_P) = -1;
        fprintf('Current point Serving range are fininshed.2\n');
    end
    
   %% Are all pixels fulfilled the constraint?
    [lossdB,User_Served,User_Covered,User_Arc] = Deploy_Result(GW_Pathloss_perPixel,CoverRange_perPixel,Tx_ind,Rxr,Rxc,TxP_Thres,GW_Serve_Limit);
    All_Utility = Constraint_Fulfilled([1:1:size(Rxc,1)],User_Served,User_Arc);
    if All_Utility >= 1 || length(find(Finish_P==0))==0
        fprintf('All points are finished\n');
        break;
    end
end
GW_Num = q_num;
filename = [floor_plan(1:length(floor_plan)-4),'_D',num2str(den)];
save(filename,'Density','lossdB','Tx_ind','Tx_Record','User_Covered','User_Served','User_Arc','GW_Serve_Limit');
%% Applying color map
originalFloorPlan = ~imdilate(~floorPlanBW,strel('disk',2));
smallFSPLImage = (reshape(lossdB,meshNode.vert.num, meshNode.horz.num));
FSPLFullImage = (imresize(smallFSPLImage,[size(floorPlan,1),size(floorPlan,2)],'method','cubic'));
FSPLFullImage = mat2gray(FSPLFullImage);
figure('Name',['Path loss method ']);
z = imoverlay(FSPLFullImage,~originalFloorPlan,[0,0,0]);
imshow(rgb2gray(z));
% text(Rxc,Rxr,num2str(int32(nodeDistance)),'FontSize',7);
% text(Rxc,Rxr,num2str([1:1:size(Rxc,1)]'),'FontSize',7);
text(Rxc,Rxr,num2str(User_Served(:,1)),'FontSize',10);
%         text(Rxc,Rxr,num2str(int32(lossdB)'),'FontSize',10);
colormap(gca,'jet');

for i = 1:7
    colorbarLabels(i) = min(lossdB) + i .* ((max(lossdB)-min(lossdB))./7);
end
colorbar('YTickLabel',num2str(int32(colorbarLabels')));
for i=1:1:size(Rxr,1)
    if Tx_ind(i) == 1
        text(Rxc(i),Rxr(i),'*','Color','Black','FontSize',30);
    end
end
title(['D=',num2str(Density),' Range=',num2str(Range),' GW=',num2str(GW_Num)]);


