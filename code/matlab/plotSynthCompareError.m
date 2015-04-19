%%
labels = {};
errors = {};
colors = [];

if(exist('errorFocalBAFixed','var'))
    fixedBA.errorFocal = errorFocalBAFixed;
    fixedBA.errorP0 = errorP0BAFixed;
    fixedBA.errorDist0 = errorDist0BAFixed;
    fixedBA.errorDist1 = errorDist1BAFixed;
    labels{end+1} = 'Best calib';
    errors{end+1} = fixedBA;
    colors(end+1) = 'k';
end

if(exist('errorFocalBA','var'))
    planarBA.errorFocal = errorFocalBA;
    planarBA.errorP0 = errorP0BA;
    planarBA.errorDist0 = errorDist0BA;
    planarBA.errorDist1 = errorDist1BA;
    
    labels{end+1} = 'Best self';
    errors{end+1} = planarBA;
    colors(end+1) = 'g';
end

if(exist('errorFocal','var'))
    ours.errorFocal = errorFocal;
    ours.errorP0 = errorP0;
    ours.errorDist0 = errorDist0;
    ours.errorDist1 = errorDist1;

    labels{end+1} = 'Ours';
    errors{end+1} = ours;
    colors(end+1) = 'b';
end

if(exist('errorFocalNoNorm','var'))
    noNorm.errorFocal = errorFocalNoNorm;
    noNorm.errorP0 = errorP0NoNorm;
    noNorm.errorDist0 = errorDist0NoNorm;
    noNorm.errorDist1 = errorDist1NoNorm;
    
    labels{end+1} = 'Self-calib no normalization';
    errors{end+1} = noNorm;
    colors(end+1) = 'r';
end

% labels = {'Ours', 'Best self-calib', 'Best calib'};
% errors = {ours, planarBA, fixedBA};
% colors = 'bgk';

keys = unique(errorKey);
%%
clf;
for kk=1:length(labels)
    err = errors{kk};
    
    %validMask = abs(err.errorFocal)<60 & ~isinf(errorDist0);
    validMask = ~isinf(errorDist0) & abs(err.errorFocal) < 1e3;
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
        errorFocalKeyed(k)=rms(m) / 600 * 100;
        m=err.errorP0(validKeys==keys(k)); 
        errorP0Keyed(k)=rms(m)/280 * 100;
        m=err.errorDist0(validKeys==keys(k)); 
        errorDist0Keyed(k)=rms(m)/0.1 * 100;
        m=err.errorDist1(validKeys==keys(k)); 
        errorDist1Keyed(k)=rms(m)/0.01 * 100;
    end

    lineStyle=['-' colors(kk)];

    subrows=2;
    subcols=2;
    subplot(subrows,subcols,1);
    hold on
    plot(keys,errorFocalKeyed,lineStyle,'DisplayName',labels{kk})
    title('Focal error (%)')
%     xlabel(errorKeyName)

    %This for angles
    %xlim([0,45])
    %l = get(gca,'XTickLabel');
    %extraL = '°';
    %l(:,end+1:end+length(extraL)) = repmat(extraL,size(l,1),1);
    %set(gca,'XTickLabel',l)
    
    subplot(subrows,subcols,2);
    hold on
    plot(keys,errorP0Keyed,lineStyle,'DisplayName',labels{kk})
    title('Principal point p_0 error (%)')
%     xlabel(errorKeyName)
    %set(gca,'XTickLabel',l)

    subplot(subrows,subcols,3);
    hold on
    plot(keys,errorDist0Keyed,lineStyle,'DisplayName',labels{kk})
    title('Distortion d_0 error (%)')
    xlabel(errorKeyName)

    subplot(subrows,subcols,4);
    hold on
    plot(keys,errorDist1Keyed,lineStyle,'DisplayName',labels{kk})
    title('Distortion d_1 error (%)')
    xlabel(errorKeyName)
    %xlim([0,45])
    
    fprintf('Sum failed calibrations for %s: %d\n', labels{kk}, sum(invalidCount));
%     xlabel(errorKeyName)

%     subplot(subrows,subcols,5);
%     hold on
%     plot(keys,invalidCount,lineStyle,'DisplayName',labels{kk})
%     title('Missed calibrations count')
%     xlabel(errorKeyName)
end
