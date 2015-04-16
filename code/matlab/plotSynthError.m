noiseKeys = unique(noise);

validMask = abs(errorFocal)<400;
noise = noise(validMask);
errorFocal = errorFocal(validMask);
errorP0 = errorP0(validMask);
errorDist0 = errorDist0(validMask);
errorDist1 = errorDist1(validMask);

errorFocalKeyed = zeros(1,length(noiseKeys));
errorP0Keyed = zeros(1,length(noiseKeys));
errorD0Keyed = zeros(1,length(noiseKeys));
errorD1Keyed = zeros(1,length(noiseKeys));
for k=1:length(noiseKeys); 
    m=errorFocal(noise==noiseKeys(k)); 
    errorFocalKeyed(k)=rms(m) / 600;
    m=errorP0(noise==noiseKeys(k)) / 280; 
    errorP0Keyed(k)=rms(m);
    m=errorDist0(noise==noiseKeys(k))/ 0.1;  
    errorD0Keyed(k)=rms(m);
    m=errorDist1(noise==noiseKeys(k))/ 0.01; 
    errorD1Keyed(k)=rms(m);
end

lineStyle='-k';

subplot(2,2,1);
hold on
plot(noiseKeys,errorFocalKeyed,lineStyle)
title('Focal error (%)')

subplot(2,2,2);
hold on
plot(noiseKeys,errorP0Keyed,lineStyle)
title('P0 error (%)')

subplot(2,2,3);
hold on
plot(noiseKeys,errorD0Keyed,lineStyle)
title('D0error (%)')

subplot(2,2,4);
hold on
plot(noiseKeys,errorD1Keyed,lineStyle)
title('D1 error (%)')