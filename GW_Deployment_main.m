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
Pixel_Res = 1;    %pixel=1m*1m
pathLossModel           = 2;        % 1=Free-Space, 2=our pathloss
TriangleDemo                = 1;        % Showes furtther details if 1

%% Pathloss Setting Parameter
TxPower                 = -63.8;        % dBm or dB
BER2SNR = @(BER)((erfcinv(2*BER))^2)*2;
SNR_Thres = BER2SNR(10^(-3));
noise = -94; %dbm
TxP_Thres = SNR_Thres + noise ;
Data_Rate = (2*(256)*8)/1000; % kbps
Max_throughput = 113; %kbps(BER=10^-3)
Control_Period = 294.7580; %ms
Density = 0.1;
% GW_Serve_Limit = floor((Max_throughput*(1000-Control_Period)/1000)/Data_Rate/3);
GW_Serve_Limit = 2;
PdBm_to_Pw = @(PdBm) 10^(PdBm/10);
Pw_to_PdBm = @(Pw) 10*log10(Pw);


folder = 'data/SK_wirelessinsite/';
floor_plan = [folder,'floorplan3.jpg'];
density_plan = [folder,'density2.jpg'];
avaliable_plan = [folder,'avaliable.jpg'];

wall_detect = [folder,'wall_detect3.mat'];
Pixel_Setting = [folder,'pixel_Setting.mat'];
Pathloss_Distance = [folder,'PlandDis_perpixel3.mat'];

% load floor plan
floorPlan = imread(floor_plan);
floorPlanBW = im2bw(floorPlan);
originalFloorPlan = floorPlanBW;
if exist(density_plan, 'file')
    ImDensity = imread(density_plan);
    ImDensityBW = ~im2bw(ImDensity);
else
    ImDensityBW = ~floorPlanBW;
end
if exist(avaliable_plan, 'file')
    ImAvaliable = imread(avaliable_plan);
    ImAvaliableBW = ~im2bw(ImAvaliable);
else
    ImAvaliableBW = ~floorPlanBW;
end

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
        if i*Unit-Unit/2 < size(floorPlanBW,2)
            horz =round(i*Unit - Unit/2);
        else
            %             horz = size(floorPlanBW,2)-1;
        end
        for j=1:1:meshNode.vert.num
            
            if j*Unit-Unit/2  < size(floorPlanBW,1)
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
        if ImDensityBW(Rxr(i),Rxc(i)) == 1 && floorPlanGray(Rxr(i),Rxc(i)) == 0
            % 表示此區域不是佈建區域
            Rxr(i) = -1;
            Rxc(i) = -1;
        else
            if floorPlanGray(Rxr(i),Rxc(i))~=0
                % 表示此區域為牆壁
                while(1)
                    t_Rxc = Rxc(i) + round(rand(1)*2-1);
                    t_Rxr = Rxr(i) + round(rand(1)*2-1);
                    if(floorPlanGray(t_Rxr,t_Rxc) == 0 && ImDensityBW(t_Rxr,t_Rxc) == 0)
                        Rxr(i) = t_Rxr;
                        Rxc(i) = t_Rxc;
                        break;
                    end
                end
            end
        end
    end
%     Rxr = Rxr(find(Rxr~=-1));
%     Rxc = Rxc(find(Rxc~=-1));
    save(Pixel_Setting,'pathUnit','meshNode','Rxc','Rxr');
end

if exist(Pathloss_Distance,'file')
    load(Pathloss_Distance);
else
    %% Calculate_Pathloss
    Tx_ind = ones(1,size(Rxc,1));
    [GW_Pathloss_perPixel , isWall_perPixel] = Calculate_Pathloss(originalFloorPlan,floorPlanGray,wallAt,pathLossModel,Tx_ind,Rxc,Rxr,pathUnit,TxP_Thres);
    %% Calculate_Distance
    Distance_perPixel = zeros(size(Rxc,1),size(Rxc,1));
    for i=1:1:size(Rxc,1)
        for j=1:1:size(Rxc,1)
            Distance_perPixel(i,j) = sqrt((Rxc(i)-Rxc(j))^2+(Rxr(i)-Rxr(j))^2);
        end
    end
    save(Pathloss_Distance,'GW_Pathloss_perPixel','Distance_perPixel','isWall_perPixel');
end
Density_map = zeros(1,size(Rxc,1));
Avaliable_map = zeros(1,size(Rxc,1));
for i=1:1:size(Rxc,1)
    Density_map(i)=Density;
     if ImDensityBW(Rxr(i),Rxc(i)) ==1
         Density_map(i) =0 ;
     end
     Avaliable_map(i) = ~ImAvaliableBW(Rxr(i),Rxc(i));
    %     if ImDensity2(Rxr(i),Rxc(i)) ==1
%         Density_map(i) =0.07 ;
%     else
%         Density_map(i) =0.124 ;
%     end
end

if TriangleDemo ==1
    F_tri =figure;
    originalFloorPlan2 =  ~imdilate(~originalFloorPlan,strel('disk',2));
    imshow(originalFloorPlan2);
%     text(Rxc,Rxr,num2str([1:1:size(Rxc,1)]'),'FontSize',7);
    hold on;
end

%% Initialization
imshow(~floorPlanGray);
text(Rxc,Rxr,num2str([1:1:size(Rxc,1)]'),'Color','red','FontSize',5);
% initial_Tx = [73,451,805,904,1,24,1213,1250,1259];
% initial_Tx = [512,671,827,908,451,1,24,1250,73,72,192,329,79,80,335,86,336,88,342,503,813,505,819,454,805,812,511,820,500,913,975,1049,1032,1077,1094,1134,1104,1161,1178,1206,1213,1168,240,360,402,531,681,570,720,804,837,903,968,923,985,1025,1059,1087,1223,1259];
initial_Tx = [1,76,8,77,9,12,84,121,132];
% initial_Tx = [1,meshNode.vert.num,meshNode.vert.num*((meshNode.horz.num)-1)+1,meshNode.vert.num*(meshNode.horz.num)];

Tx_ind = zeros(1,size(Rxc,1));
Queue_ind = zeros(1,size(Rxc,1));
Tx_ind(initial_Tx) =1;
q_num = length(find(Tx_ind==1));
GW_Num = length(find(Tx_ind==1));
Queue_ind(initial_Tx) = [1:1:GW_Num];
Finish_P = zeros(1,size(Rxc,1));
Range = 10;
Utility_Tres = 1;
text(Rxc(initial_Tx),Rxr(initial_Tx),'*','Color','blue','FontSize',30);
%% Calculate Covering range
CoverRange_perPixel = zeros(size(Rxc,1),size(Rxc,1));
for i=1:1:size(Rxc,1)
    Covered1_P = zeros(1,size(Rxc,1));
    Covered2_P =zeros(1,size(Rxc,1));
    [Sort_P1,ind_P1] = sort(GW_Pathloss_perPixel(i,:),'descend');
    ind_P1 = ind_P1(GW_Pathloss_perPixel(i,ind_P1)>=TxP_Thres);
    Covered1_P = Calculate_Range(Density_map,Sort_P1,ind_P1,GW_Serve_Limit*Range);
    
    [Sort_DP,ind_DP] = sort(Distance_perPixel(i,:));
    ind_DP = ind_DP(GW_Pathloss_perPixel(i,ind_DP)>=TxP_Thres);
    Covered2_P = Calculate_Range(Density_map,Sort_DP,ind_DP,GW_Serve_Limit*Range);

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
%      CurP_Served1 = Calculate_Range(Density_map,Sort_P1,ind_p1,GW_Serve_Limit);

    [Sort_D,ind_D] = sort(Distance_perPixel(Cur_P,:));
    ind_D = ind_D(find(GW_Pathloss_perPixel(Cur_P,ind_D)>=TxP_Thres));
    CurP_Served2 = Calculate_Range(Density_map,Sort_D,ind_D,GW_Serve_Limit);
%     CurP_Served2 = Calculate_ServingRange(Density_map,Sort_D,ind_D,GW_Serve_Limit,TxP_Thres);

    CurP_Served = (CurP_Served1 | CurP_Served2);
    CurP_Covered = find(CoverRange_perPixel(Cur_P,:)==1);
%     tic
    [lossdB,User_Served,User_Covered,User_Arc] = Deploy_Result(GW_Pathloss_perPixel,CoverRange_perPixel,Tx_ind,Rxr,Rxc,TxP_Thres,GW_Serve_Limit,Density_map);
    Tri_Utility = Constraint_Fulfilled([1:1:size(Rxc,1)],User_Served,User_Arc);
%     toc
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
                    if TriangleDemo == 2
                        % draw the triangle
                        x= [Rxc(Cur_P),Rxc(Candidate_P(j)),Rxc(Link(k)),Rxc(Cur_P)];
                        y= [Rxr(Cur_P),Rxr(Candidate_P(j)),Rxr(Link(k)),Rxr(Cur_P)];
                        plot(x,y);
                    end
                end
            end
        end
    end
     
    %% Is GW_cur’s serving range (S_cur) fulfilled the constraint?
    CurP_Utility = Constraint_Fulfilled(find(CurP_Served==1),User_Served,User_Arc);
    if TriangleDemo == 2
        figure;
        imshow(floorPlanBW);
        text(Rxc(Tx_ind==1),Rxr(Tx_ind==1),'*','Color','green','FontSize',20);
        text(Rxc(CoverRange_perPixel(Cur_P,:)==1),Rxr(CoverRange_perPixel(Cur_P,:)==1),'o','Color','Black','FontSize',10);
        text(Rxc(CurP_Served==1),Rxr(CurP_Served==1),'x','Color','red','FontSize',10);

    end
    if CurP_Utility < Utility_Tres || length(find((CurP_Served.*(Finish_P|Tx_ind))==1)) < length(find((CurP_Served==1)))
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
                    Candidate_P = intersect(Candidate_P,find(Avaliable_map==1));
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
                            [~,try_User_Served,try_User_Covered,try_User_Arc] = Put_OneGW(GW_Pathloss_perPixel,CoverRange_perPixel,Candidate_P(j),Rxc,Rxr,User_Served,User_Arc,User_Covered,lossdB,TxP_Thres,GW_Serve_Limit,Density_map);
                            % the fitness of all the pixels
                            Fitness = Calculate_Fitness([1:1:size(Rxc,1)],try_User_Arc);
                            Cur_Utility = Constraint_Fulfilled(find(CurP_Served==1),try_User_Served,try_User_Arc);
                            Tri_Utility = Constraint_Fulfilled(find(InsideTri_P==1),try_User_Served,try_User_Arc);
                            Fitness =  (Fitness)*(length(find((InsideTri_P.*CurP_Served)==1))-length(find((InsideTri_P.*Finish_P.*CurP_Served)==1)));
                            % best utility or current point 's served range is finished
                            if ((Fitness > Temp_Fitness) || Cur_Utility >= Utility_Tres )&& Tri_Utility==1
                                if Cur_Utility >=Utility_Tres
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
                    fprintf('Cur_P = %d , Best_Link = %d , Best_P = %d ,Utility = %f\n',Cur_P,Best_Link,Best_P,Best_Fitness);
                    if TriangleDemo ==1
                        x= [Rxc(Cur_P),Rxc(Best_Link),Rxc(Best_P),Rxc(Cur_P)];
                        y= [Rxr(Cur_P),Rxr(Best_Link),Rxr(Best_P),Rxr(Cur_P)];
                        plot(x,y);
                    end
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
                [lossdB,User_Served,User_Covered,User_Arc] = Deploy_Result(GW_Pathloss_perPixel,CoverRange_perPixel,Tx_ind,Rxr,Rxc,TxP_Thres,GW_Serve_Limit,Density_map);
                Cur_Utility = Constraint_Fulfilled(find(CurP_Served==1),User_Served,User_Arc);
                if Cur_Utility >= Utility_Tres
                    break;
                end
            end
        end
        if (length(Link) == 0 || Best_Fitness == 0)
            Candidate_P =  find(CoverRange_perPixel(Cur_P,:)==1);
            Candidate_P = intersect(Candidate_P,find(Avaliable_map==1));
            Best_Fitness = 0;
            for j=1:1:length(Candidate_P)
                if Finish_P(Candidate_P(j)) ~= 1 && Tx_ind(Candidate_P(j))~=1
                    temp_Tx_ind = Tx_ind;
                    temp_Tx_ind(Candidate_P(j)) =1;
                    [try_lossdB,try_User_Served,try_User_Covered,try_User_Arc] = ...
                        Deploy_Result(GW_Pathloss_perPixel,CoverRange_perPixel,temp_Tx_ind,Rxr,Rxc,TxP_Thres,GW_Serve_Limit,Density_map);
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
                fprintf('Cur_P = %d , Best_P = %d , Utility = %f \n',Cur_P,Best_P,Best_Fitness);
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
    [lossdB,User_Served,User_Covered,User_Arc] = Deploy_Result(GW_Pathloss_perPixel,CoverRange_perPixel,Tx_ind,Rxr,Rxc,TxP_Thres,GW_Serve_Limit,Density_map);
    All_Utility = Constraint_Fulfilled([1:1:size(Rxc,1)],User_Served,User_Arc);
    if All_Utility >= Utility_Tres || length(find(Finish_P==0))==0
        fprintf('All points are finished\n');
        break;
    end
end
GW_Num = q_num;
subfolder = datestr(now,30);
mkdir([folder,subfolder]);
filename = [folder,subfolder,'/DeployResult.mat'];
save(filename,'floor_plan','density_plan','avaliable_plan','wall_detect','Pixel_Setting','Pathloss_Distance','initial_Tx','TxP_Thres','lossdB','Tx_ind','User_Covered','User_Served','User_Arc','GW_Serve_Limit','Avaliable_map','Density_map','Range','GW_Num');
%% Applying color map
% originalFloorPlan = ~imdilate(~floorPlanBW,strel('disk',2));
% smallFSPLImage = (reshape(lossdB,meshNode.vert.num, meshNode.horz.num));
% FSPLFullImage = (imresize(smallFSPLImage,[size(floorPlan,1),size(floorPlan,2)],'method','cubic'));
% FSPLFullImage = mat2gray(FSPLFullImage);
% figure('Name',['Path loss method ']);
% z = imoverlay(FSPLFullImage,~originalFloorPlan,[0,0,0]);
% imshow(rgb2gray(z));
% % text(Rxc,Rxr,num2str(int32(nodeDistance)),'FontSize',7);
% % text(Rxc,Rxr,num2str([1:1:size(Rxc,1)]'),'FontSize',7);
% text(Rxc,Rxr,num2str(User_Served(:,1)),'FontSize',10);
% %         text(Rxc,Rxr,num2str(int32(lossdB)'),'FontSize',10);
% colormap(gca,'jet');
% 
% for i = 1:7
%     colorbarLabels(i) = min(lossdB) + i .* ((max(lossdB)-min(lossdB))./7);
% end
% colorbar('YTickLabel',num2str(int32(colorbarLabels')));
% for i=1:1:size(Rxr,1)
%     if Tx_ind(i) == 1
%         text(Rxc(i),Rxr(i),'*','Color','Black','FontSize',30);
%     end
% %     if CoverRange_perPixel(Cur_P,i)==1
% %         text(Rxc(i),Rxr(i),'o','Color','Black','FontSize',10);
% %     end
% end
% title(['FeasibleDensity, Range=',num2str(Range),' GW=',num2str(GW_Num)]);

figure;
imshow(floor_plan);
% text(Rxc,Rxr,num2str(User_Served(:,1)),'FontSize',10);
for i=1:1:size(Rxr,1)
    if Tx_ind(i) == 1
        text(Rxc(i),Rxr(i),'*','Color','Black','FontSize',20);
    end
end
title(['FeasibleDensity, Range=',num2str(Range),' GW=',num2str(GW_Num)]);
saveas(gcf,[folder,subfolder,'/DeployResult.png']);
 [ lossdB,Tx_ind,User_Covered,User_Served,User_Arc,GW_Num ] = Merge_Algorithm(folder,0,filename,Range);
Merge_filename =[folder,subfolder,'/DeployResult_Merge.mat'];
save(Merge_filename,'floor_plan','density_plan','avaliable_plan','wall_detect','Pixel_Setting','Pathloss_Distance','initial_Tx','TxP_Thres','lossdB','Tx_ind','User_Covered','User_Served','User_Arc','GW_Serve_Limit','Avaliable_map','Density_map','Range','GW_Num');
figure;
imshow(floor_plan);
% text(Rxc,Rxr,num2str(User_Served(:,1)),'FontSize',10);
for i=1:1:size(Rxr,1)
    if Tx_ind(i) == 1
        text(Rxc(i),Rxr(i),'*','Color','Black','FontSize',20);
    end
end
title(['FeasibleDensity, Range=',num2str(Range),' GW=',num2str(GW_Num)]);