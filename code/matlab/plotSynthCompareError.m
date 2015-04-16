ours = [];
ours.errorFocal = errorFocal;
ours.errorP0 = errorP0;
ours.errorDist0 = errorDist0;
ours.errorDist1 = errorDist1;

planarBA.errorFocal = errorFocalBA;
planarBA.errorP0 = errorP0BA;
planarBA.errorDist0 = errorDist0BA;
planarBA.errorDist1 = errorDist1BA;

fixedBA.errorFocal = errorFocalBAFixed;
fixedBA.errorP0 = errorP0BAFixed;
fixedBA.errorDist0 = errorDist0BAFixed;
fixedBA.errorDist1 = errorDist1BAFixed;

labels = {'Ours', 'Best self-calib', 'Best calib'};
errors = {ours, planarBA, fixedBA};
colors = 'bgk';

keys = unique(errorKey);

clf;
for kk=1:length(labels)
    err = errors{kk};
    
    validMask = abs(err.errorFocal)<400;
    validKeys = errorKey(validMask);
    err.errorFocal = err.errorFocal(validMask);
    err.errorP0 = err.errorP0(validMask);
    err.errorDist0 = err.errorDist0(validMask);
    err.errorDist1 = err.errorDist1(validMask);

    invalidCount = zeros(1,length(keys));    
    errorFocalKeyed = zeros(1,length(keys));
    errorP0Keyed = zeros(1,length(keys));
    errorDist0Keyed = zeros(1,length(keys));
    errorDist1Keyed = zeros(1,length(keys));
    for k=1:length(keys); 
        invalidCount(k) = sum(~validMask(errorKey==keys(k)));
        m=err.errorFocal(validKeys==keys(k)); 
        errorFocalKeyed(k)=rms(m) / 600;
        m=errorP0(validKeys==keys(k)); 
        errorP0Keyed(k)=rms(m)/280;
        m=errorDist0(validKeys==keys(k)); 
        errorDist0Keyed(k)=rms(m)/0.1;
        m=errorDist1(validKeys==keys(k)); 
        errorDist1Keyed(k)=rms(m)/0.01;
    end

    lineStyle=['-' colors(kk)];

    subplot(2,3,1);
    hold on
    plot(keys,errorFocalKeyed,lineStyle)
    title('Focal error (%)')
    xlabel(errorKeyName)

    subplot(2,3,2);
    hold on
    plot(keys,errorP0Keyed,lineStyle)
    title('P0 error (%)')
    xlabel(errorKeyName)

    subplot(2,3,3);
    hold on
    plot(keys,errorDist0Keyed,lineStyle)
    title('D0error (%)')
    xlabel(errorKeyName)

    subplot(2,3,4);
    hold on
    plot(keys,errorDist1Keyed,lineStyle)
    title('D1 error (%)')
    xlabel(errorKeyName)

    subplot(2,3,5);
    hold on
    plot(keys,invalidCount,lineStyle)
    title('Missed calibrations count')
    xlabel(errorKeyName)
end
