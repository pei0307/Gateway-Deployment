function GW_Pathloss_perPixel = Calculate_Pathloss (originalFloorPlan,floorPlanGray,wallAt,pathLossModel,Tx_ind,Rxc,Rxr,pathUnit,TxP_Thres)

TxPower                 = -63.8;        % dBm or dB
antennaLoss             = 0;        % dB
TxAntennaGain           = 0 + antennaLoss ; % Gain of Transmitting antenna
RxAntennaGain           = TxAntennaGain;    % Gain of Receiving antenna

GW_Pathloss_perPixel = zeros(length(Rxr),length(Rxr));
Tx_ind = find(Tx_ind == 1);
for d=1:1:length(Tx_ind)
    fprintf('d = %d\n',d);
    %% Calculating mesh points distance from Tx
    dRxTxr = Rxr - Rxr(Tx_ind(d)); % distance in terms of pixels
    dRxTxc = Rxc - Rxc(Tx_ind(d)); % distance in terms of pixels
    nodeDistance = sqrt(dRxTxr.^2 + dRxTxc.^2) * pathUnit; % distance in terms of meters
    
    %% Indoor Propagation Models
    % 1-  FREE SPACE PATH LOSS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if pathLossModel == 1
        Temp_loss = 10*0.72*log10(nodeDistance);
        %         lossdB = 20 * log10(4*pi.*nodeDistance.*freq./lightVel) .* (nodeDistance > d0FSPL) + (nodeDistance < d0FSPL) .* 20 * log10(4*pi.*d0FSPL.*freq./lightVel);
        %         loss = (4*pi.*nodeDistance*freq./lightVel).^2;
        Temp_loss = TxPower - Temp_loss + RxAntennaGain + TxAntennaGain;
    end
    % % 2- COST 231 Model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if pathLossModel == 2
        Temp_loss = zeros(1,size(Rxc,1));
        % % LOS & Walls Determination
        % Thining the floor plan. Only one pixel per wall should intersect with LOS
        thinFloorPlanBW = ~ originalFloorPlan;
        thinFloorPlanBW = bwmorph(thinFloorPlanBW,'thin','inf');
        thinFloorPlanBW = bwmorph(thinFloorPlanBW,'diag');
        
        losC = cell(size(Rxr,1),1);
        losR = cell(size(Rxr,1),1);
        
        losTemp = zeros(size(thinFloorPlanBW));
        wallsType = cell(size(Rxr,1),1); % pre-defining
        numWalls = zeros(size(Rxr,1),1);
        
        for i = d:numel(Rxr)
            [losC{i},losR{i}] = bresenham(Rxc(Tx_ind(d)),Rxr(Tx_ind(d)),Rxc(i),Rxr(i)); %LOS between Tx &Rx
            for j = 1:numel(losC{i}(:))
                losTemp(losR{i}(j),losC{i}(j)) = 1; % temporary line of sight image
            end
            [wallsLable,numWalls(i)] = bwlabel(losTemp .* thinFloorPlanBW,8); % find intersection of LOS and walls
            wallsLable = bwmorph(wallsLable,'shrink','inf');
            wallsType{i} = unique(double(floorPlanGray).*wallsLable);  % type of the walls (grayscale) between each Tx to Rx
            
            % calculating the total wall attenuation for each beam
            wallLoss = 0;
            for k = 2:numel(wallsType{i})
                wallLoss =  wallAt(wallsType{i}(k)) + wallLoss;
            end
            % Calculating the signal strength
            %         lossdB(i) = ((20 * log10(4*pi.*nodeDistance(i).*freq./lightVel) .* (nodeDistance(i) > d0Cost231)) + ((nodeDistance(i) < d0Cost231) .* 20 * log10(4*pi.*d0Cost231.*freq./lightVel)) ) ...
            if nodeDistance(i) ~=0
                Temp_loss(i) = (nodeDistance(i) ~=0)*10*0.72*log10(nodeDistance(i))+ abs(wallLoss);
            else
                Temp_loss(i) = -3;
            end
            Temp_loss(i) = TxPower - Temp_loss(i) + RxAntennaGain + TxAntennaGain;
            
            losTemp = zeros(size(thinFloorPlanBW)); % clears the LOS image
        end
    end
    GW_Pathloss_perPixel(Tx_ind(d),:) = Temp_loss;
    GW_Pathloss_perPixel = GW_Pathloss_perPixel+(GW_Pathloss_perPixel');
    for i=1:1:size(Rxc,1)
        GW_Pathloss_perPixel(i,i) = 60.8;
    end
end
