loadDataIntoWorkspace = 0;
if loadDataIntoWorkspace == 1
  clear all; close all; clc

  gpsData = csvread('sortedGPGGA.csv', 1, 0);

  %time starting from zero
  t = gpsData(:,4);
  
  timeInHours = 1;
  timeInMinutes = 0;
  if timeInHours == 1
    t = t/3600;
  end
  if timeInMinutes == 1
    t = t/60;
  end

  %latitude and longitude converted to decimal degress
  latitude = gpsData(:,5)+gpsData(:,6)./60;
  longitude = gpsData(:,7)+gpsData(:,8)./60;
  
  %---------------------------------------------------------------------
  %---CALCULATING THE DISTANCES xd AND yd FROM LATITUDE AND LONGITUDE---
  %---------------------------------------------------------------------

  R = 6363128; %6371000  %<--radius of the earth
  
  %Convert latitude and longitude to radians
  latRads = deg2rad(latitude);
  lngRads = deg2rad(longitude);
  
  %Choose Mean:
  singleMean = 0;
  fixMean = 1;
  %computing mean as center to be mesured from
  if singleMean == 1
    latRadsMean = mean(latRads(1:9834));
    lngRadsMean = mean(lngRads(1:9834)); 
  elseif fixMean == 1
    latRadsMean = mean(latRads(4941:46345));
    lngRadsMean = mean(lngRads(4941:46345)); 
  end
  
  
    %latRadsMean = mean(latRads);
    %lngRadsMean = mean(lngRads); 

  %Compute difference from mean in lattitude and longitude
  diffLat = latRads - latRadsMean;
  diffLng = lngRads - lngRadsMean;

  %Compute trigonomtric functions
  cosLat0 = cos(latRadsMean);
  sinLat0 = sin(latRadsMean);
  cosLng0 = cos(lngRadsMean);
  sinLng0 = sin(lngRadsMean);
  cosLat1 = cos(latRads);
  sinLat1 = sin(latRads);
  cosLng1 = cos(lngRads);
  sinLng1 = sin(lngRads);
  sinLat10 = sin((diffLat)./2);

  %Compute distance from mean in xn directions
  dx = R*2*atan2(sinLat10,sqrt(1-sinLat10.*sinLat10));

  %Compute total distance to the origin in NED frame
  x1 = R * cosLat1 .* cosLng1;
  y1 = R * cosLat1 .* sinLng1;
  z1 = R * sinLat1;

  x0 = R * cosLat0 .* cosLng0;
  y0 = R * cosLat0 .* sinLng0;
  z0 = R * sinLat0;

  xd = x1 - x0;
  yd = y1 - y0;
  zd = z1 - z0;

  d = sqrt(xd .* xd + yd .* yd + zd .* zd);

  %Compute distance in yn directions
  dy = sqrt(d .* d - dx .* dx);

  %Put the correct sign
  for i =1:length(dy)
    if diffLng(i) < 0
      dy(i) = -dy(i);
    end
  end
  
%-----------------------------------------------------------------------
%-----------------------------------------------------------------------

useDistance = 1;
if useDistance == 1
  latitude = dx;
  longitude = dy;
end

%altitude as is (in meters)
  altitude = gpsData(:,10);

  %this value tels wether the GPS has base corrections
  %and to which degree it udses them
  gpsFixQuality = gpsData(:,9);

  %Fix quality:
  %   0 = invalid
  %   1 = GPS fix (SPS)
  %   2 = DGPS fix
  %   3 = PPS fix
  %   4 = Real Time Kinematic
  %   5 = Float RTK
  %   6 = estimated (dead reckoning) (2.3 feature)
  %   7 = Manual input mode

  si = 0;
  fi = 0;
  fl = 0;
  ot = 0;

  %sort data depending on fix
  for i=1:length(gpsData)
    if gpsFixQuality(i) == 1
      si = si+1;
      tSingle(si)      = t(i);
      latSingle(si)    = latitude(i);
      lngSingle(si)    = longitude(i);
      altSingle(si)    = altitude(i);
    elseif gpsFixQuality(i) == 4
      fi = fi+1;
      tRTKfix(fi)      = t(i);
      latRTKfix(fi)    = latitude(i);
      lngRTKfix(fi)    = longitude(i);
      altRTKfix(fi)    = altitude(i);
    elseif gpsFixQuality(i) == 5
      fl = fl+1;
      tRTKfloat(fl)    = t(i);
      latRTKfloat(fl)  = latitude(i);
      lngRTKfloat(fl)  = longitude(i);
      altRTKfloat(fl)  = altitude(i);
    else
      ot = ot+1
      tOther(ot)       = t(i);
      latOther(ot)     = latitude(i);
      lngOther(ot)     = longitude(i);
      altOther(ot)     = altitude(i);
    end
    clc
    disp('Number of:')
    fprintf('singles: %d floats: %d fixes: %d others: %d\n', si, fl, fi, ot)
    
    %plot points progressively (very slow)
    enableProgress = 0;
    if enableProgress == 1
      if fi > 1
        plot3(latRTKfix(fi-1:fi),lngRTKfix(fi-1:fi),altRTKfix(fi-1:fi),'b')
        hold on
      end
      if fl > 1
        plot3(latRTKfloat(fl-1:fl),lngRTKfloat(fl-1:fl),altRTKfloat(fl-1:fl),'r')
        hold on
      end
      if si > 1
        plot3(latSingle(si-1:si),lngSingle(si-1:si),altSingle(si-1:si),'g')
        hold on
      end
      drawnow
    end
  end
else
  clc; close all;
end


%-----------------------------------------------------------------------
%------PLOT OPTIONS-----------------------------------------------------
%-----------------------------------------------------------------------


%select a range in time to plot
plotRange = 0;
plotStartTime = 0.1;  %1.5
plotEndTime   = 5.5; %3.8

%3D-plot options
plotAllLines3D = 0;
plotAllDots3D = 1;
lineWidth3D = 1;
dotSize3D = 1.2;

%2D-plot options
plotAll2D = 1;
useSubPlots = 1;
plotAllLines2D = 0;
plotAllDots2D = 1;
lineWidth2D = 1;
dotSize2D = 1.5; 

%zoom options for 2D-plot
zoomOnFix = 0;
zoomOnFloat = 0;
zoomOnSingle = 0;

printLegends = 1;

%note: zoom for 3D-plot is manual, see under plot3-code

%-----------------------------------------------------------------------
%------SPECIFIC OVERRULING PLOT CHOICE----------------------------------
%-----------------------------------------------------------------------

%Plot different fix qualities together
if 1
  plotRange = 1;
  plotStartTime = 0.1031;%0.09;
  plotEndTime   = 5.5;
  
  zoomOnFix = 0;
  zoomOnFloat = 0;
  zoomOnSingle = 1;

  printLegends = 1;
  
  plotAllLines2D = 0;
  plotAllDots2D = 1;
end
%Only RKT Fix mesurements
if 0
  plotRange = 1;
  plotStartTime = 1.5
  plotEndTime   = 3.8
   
  zoomOnFix = 1;
  zoomOnFloat = 0;
  zoomOnSingle = 0;

  printLegends = 0;

  plotAllLines2D = 1;
  plotAllDots2D = 0;
end

%-----------------------------------------------------------------------
%-----------------------------------------------------------------------

%code for plotting a range of the data
if plotRange == 1
  
  low = 0;
  high = 0;

  for i = 1:length(t)
    if t(i) >= plotStartTime && low == 0;
      low = i;
    end
    if t(i) >= plotEndTime && high == 0;
      high = i;
    end
  end

  lowFix     = 0;
  lowFloat   = 0;
  lowSingle  = 0;
  for i = 1:length(tRTKfix)
    if tRTKfix(i) >= t(low) && lowFix == 0
      lowFix = i;
    end
    if tRTKfix(i) <= t(high)
      highFix = i;
    end
  end
  for i = 1:length(tRTKfloat)
    if tRTKfloat(i) >= t(low) && lowFloat == 0
      lowFloat = i;
    end
    if tRTKfloat(i) <= t(high)
      highFloat = i;
    end
  end
  for i = 1:length(tSingle)
    if tSingle(i) >= t(low) && lowSingle == 0
      lowSingle = i;
    end
    if tSingle(i) <= t(high)
      highSingle = i;
    end
  end
else
  lowFix = 1;
  highFix = length(tRTKfix);
  lowFloat = 1;
  highFloat = length(tRTKfloat);
  lowSingle = 1;
  highSingle = length(tSingle);
end

plotFixAroundZero = 1;
if plotFixAroundZero == 1
  latRTKfix(lowFix:highFix) = latRTKfix(lowFix:highFix)-mean(latRTKfix(lowFix:highFix));
  lngRTKfix(lowFix:highFix) = lngRTKfix(lowFix:highFix)-mean(lngRTKfix(lowFix:highFix));
  altRTKfix(lowFix:highFix) = altRTKfix(lowFix:highFix)-mean(altRTKfix(lowFix:highFix));
end


%see 3D plot options in top of script
if plotAllLines3D == 1
  figure
  if lowFix ~= 0
    plot3(latRTKfix(lowFix:highFix), ...
          lngRTKfix(lowFix:highFix), ...
          altRTKfix(lowFix:highFix), ...
          'color', '[0 0 1]',        ...
          'linewidth', lineWidth3D)  
    hold on
  end
  if lowFloat ~= 0
    plot3(latRTKfloat(lowFloat:highFloat), ...
          lngRTKfloat(lowFloat:highFloat), ...
          altRTKfloat(lowFloat:highFloat), ...
          'color', '[0 .5 0]',             ...
          'linewidth', lineWidth3D)
    hold on
  end
  if lowSingle ~= 0
    plot3(latSingle(lowSingle:highSingle), ...
          lngSingle(lowSingle:highSingle), ...
          altSingle(lowSingle:highSingle), ...
          'color', '[1 0 0]',              ...
          'linewidth', lineWidth3D)  
    hold on
  end
end

if plotAllDots3D == 1
  if lowFix ~= 0
    plot3(latRTKfix(lowFix:highFix), ...
          lngRTKfix(lowFix:highFix), ...
          altRTKfix(lowFix:highFix), ...
          '.',                       ...
          'color', '[0 0 1]',        ...
          'linewidth', dotSize3D)  
    hold on
  end
  if lowFloat ~= 0
    plot3(latRTKfloat(lowFloat:highFloat), ...
          lngRTKfloat(lowFloat:highFloat), ...
          altRTKfloat(lowFloat:highFloat), ...
          '.',                             ...
          'color', '[0 .5 0]',             ...
          'linewidth', dotSize3D)
    hold on
  end
  if lowSingle ~= 0
    plot3(latSingle(lowSingle:highSingle), ...
          lngSingle(lowSingle:highSingle), ...
          altSingle(lowSingle:highSingle), ...
          '.',                             ...
          'color', '[1 0 0]',              ...
          'linewidth', dotSize3D)
  end
end

%zoom on data in 3D-plot
zoom = 0;
if zoom == 1
  xlim([62.6 62.7])
  ylim([109.9 109.96])
  zlim([-28.2 -27.9])
end

title('Relative GPS Position', 'interpreter', 'latex')
xlabel('$x_\mathrm{n}$ [m]','interpreter', 'latex')
ylabel('$y_\mathrm{n}$ [m]','interpreter', 'latex')
zlabel('Altitude [m]','interpreter', 'latex')
if printLegends == 1
  leg0 = legend('RTK Fix', 'RTK float', 'Single', 'location', 'northeast');
  set(leg0, 'FontSize', 10,'interpreter', 'latex');
end
grid on; grid minor

%see 2D plot options in top of script
if plotAll2D == 1
  %-----Plot latitude over time--------------------
  if useSubPlots == 1
    figure
    sub1 = subplot(3,1,1);
  else
    figure
  end
  if plotAllLines2D == 1
    if lowFix ~= 0
      plot(tRTKfix(lowFix:highFix),    ...
           latRTKfix(lowFix:highFix),  ...
           'color', '[0 0 1]',         ...
           'linewidth', lineWidth2D)  
      hold on
    end
    if lowFloat ~= 0
      plot(tRTKfloat(lowFloat:highFloat),   ...
           latRTKfloat(lowFloat:highFloat), ...
           'color', '[0 .5 0]',             ...
           'linewidth', lineWidth2D)
      hold on
    end
    if lowSingle ~= 0
      plot(tSingle(lowSingle:highSingle),   ...
           latSingle(lowSingle:highSingle), ...
           'color', '[1 0 0]',              ...
           'linewidth', lineWidth2D)  
      hold on
    end
  end
  if plotAllDots2D == 1
    if lowFix ~= 0
      plot(tRTKfix(lowFix:highFix),         ...
           latRTKfix(lowFix:highFix),       ...
           '.',                             ...
           'color', '[0 0 1]',              ...
           'linewidth', dotSize2D)  
      hold on
    end
    if lowFloat ~= 0
      plot(tRTKfloat(lowFloat:highFloat),   ...
           latRTKfloat(lowFloat:highFloat), ...
           '.',                             ...
           'color', '[0 .5 0]',             ...
           'linewidth', dotSize2D)
      hold on
    end
    if lowSingle ~= 0
      plot(tSingle(lowSingle:highSingle),   ...
           latSingle(lowSingle:highSingle), ...
           '.',                             ...
           'color', '[1 0 0]',              ...
           'linewidth', dotSize2D)
    end
  end
  p1 = gca;
 
  %-----Plot longitude over time--------------------
  if useSubPlots == 1
    sub2 = subplot(3,1,2);
  else
    figure
  end 
  if plotAllLines2D == 1
    if lowFix ~= 0
      plot(tRTKfix(lowFix:highFix),         ... 
           lngRTKfix(lowFix:highFix),       ...
           'color', '[0 0 1]',              ...
           'linewidth', lineWidth2D)  
      hold on
    end
    if lowFloat ~= 0
      plot(tRTKfloat(lowFloat:highFloat),   ...
           lngRTKfloat(lowFloat:highFloat), ...
           'color', '[0 .5 0]',             ...
           'linewidth', lineWidth2D)
      hold on
    end
    if lowSingle ~= 0
      plot(tSingle(lowSingle:highSingle),   ...
           lngSingle(lowSingle:highSingle), ...
           'color', '[1 0 0]',              ...
           'linewidth', lineWidth2D)
      hold on
    end
  end
  if plotAllDots2D == 1
    if lowFix ~= 0
      plot(tRTKfix(lowFix:highFix),         ...
           lngRTKfix(lowFix:highFix),       ...
           '.',                        ...
           'color', '[0 0 1]',              ...
           'linewidth', dotSize2D)  
      hold on
    end
    if lowFloat ~= 0
      plot(tRTKfloat(lowFloat:highFloat),   ...
           lngRTKfloat(lowFloat:highFloat), ...
           '.',                        ...
           'color', '[0 .5 0]',             ...
           'linewidth', dotSize2D)
      hold on
    end
    if lowSingle ~= 0
      plot(tSingle(lowSingle:highSingle),   ...
           lngSingle(lowSingle:highSingle), ...
           '.',                        ...
           'color', '[1 0 0]',              ...
           'linewidth', dotSize2D)
    end
  end
  p2 = gca;
  
  %-----Plot altitude over time-------------------- 
  if useSubPlots == 1
    sub3 = subplot(3,1,3);
  else
    figure
  end 
  if plotAllLines2D == 1
    if lowFix ~= 0
      plot(tRTKfix(lowFix:highFix),         ...
           altRTKfix(lowFix:highFix),       ...
           'color', '[0 0 1]',              ...
           'linewidth', lineWidth2D)  
      hold on
    end
    if lowFloat ~= 0
      plot(tRTKfloat(lowFloat:highFloat),   ...
           altRTKfloat(lowFloat:highFloat), ...
           'color', '[0 .5 0]',             ...
           'linewidth', lineWidth2D)
      hold on
    end
    if lowSingle ~= 0
      plot(tSingle(lowSingle:highSingle),   ...
           altSingle(lowSingle:highSingle), ...
           'color', '[1 0 0]',              ...
           'linewidth', lineWidth2D)  
      hold on
    end
  end
  if plotAllDots2D == 1
    if lowFix ~= 0
      plot(tRTKfix(lowFix:highFix),         ...
           altRTKfix(lowFix:highFix),       ...
           '.',                             ...
           'color', '[0 0 1]',              ...
           'linewidth', dotSize2D)  
      hold on
    end
    if lowFloat ~= 0
      plot(tRTKfloat(lowFloat:highFloat),   ...
           altRTKfloat(lowFloat:highFloat), ...
           '.',                             ...
           'color', '[0 .5 0]',             ...
           'linewidth', dotSize2D)
      hold on
    end
    if lowSingle ~= 0
      plot(tSingle(lowSingle:highSingle),   ...
           altSingle(lowSingle:highSingle), ...
           '.',                             ...
           'color', '[1 0 0]',              ...
           'linewidth', dotSize2D)
    end
  end
  p3 = gca;
end




%-----------------------------------------------------------------------
%------COMPUTING 95% CONFIDENCE INTERVAL--------------------------------
%-----------------------------------------------------------------------

x_n = latRTKfix(lowFix:highFix);
y_n = lngRTKfix(lowFix:highFix);
z_n = altRTKfix(lowFix:highFix);

% finding sample mean of x_n and y_n
x_n_mean = mean(x_n);
y_n_mean = mean(y_n);
z_n_mean = mean(z_n);

x_n_sigma = std(x_n);
y_n_sigma = std(y_n);
z_n_sigma = std(z_n);

x_n_var = x_n_sigma^2;
y_n_var = y_n_sigma^2;
z_n_var = z_n_sigma^2;

dist2Mean = sqrt( (x_n-x_n_mean).^2 + (y_n-y_n_mean).^2 )

percentile95 = prctile(dist2Mean,95);

figure;
h1 = histogram(dist2Mean, 60, 'normalization', 'probability')

l1 = line([percentile95, percentile95], get(gca,'Ylim'))

set(l1, 'color', '[0 .5 0]')
set(l1, 'linewidth', 1.5)

title('Distance from Mean in $x_n$-$y_n$-plane', 'interpreter', 'latex')
xlabel('Discance [m]', 'interpreter', 'latex')
ylabel('Probability', 'interpreter', 'latex')

grid on, grid minor

text(.33,.855555,'95th percentile','units','normalized', 'FontWeight', 'bold', 'interpreter','latex', 'FontSize', 11, 'color', '[0 .4 0]')

fh = figure
h = histogram2(x_n,y_n,40, 'Normalization', 'probability')

limx = get(get(fh, 'Children'), 'XLim')
limy = get(get(fh, 'Children'), 'YLim')
limz = get(get(fh, 'Children'), 'ZLim')

hold on

plotMean = 0;
if plotMean == 1
  histogram2(x_n_mean, y_n_mean,10, 'Normalization', 'count','BinMethod','scott')

  set(get(fh, 'Children'), 'XLim', limx)
  set(get(fh, 'Children'), 'YLim', limy)
  set(get(fh, 'Children'), 'ZLim', [limz(1), limz(2)+10])
end

grid on, grid minor

title('RTK GPS Relative Position', 'interpreter', 'latex')
xlabel('$x_n [m]$', 'interpreter', 'latex')
ylabel('$y_n$ [m]', 'interpreter', 'latex')
zlabel('Probability', 'interpreter', 'latex')




%-----------------------------------------------------------------------
%-----------------------------------------------------------------------



if useSubPlots == 1
  plot1st = sub1;
  plot2nd = sub2;
  plot3rd = sub3;
elseif useSubPlots == 0
  plot1st = p1;
  plot2nd = p2;
  plot3rd = p3;
end

%see zoom-options at top of script
if zoomOnFix == 1
  limits1 = [ min(latRTKfix(lowFix:highFix)) max(latRTKfix(lowFix:highFix)) ];
  limits2 = [ min(lngRTKfix(lowFix:highFix)) max(lngRTKfix(lowFix:highFix)) ];
  limits3 = [ min(altRTKfix(lowFix:highFix)) max(altRTKfix(lowFix:highFix)) ];
end
if zoomOnFloat == 1
  limits1 = [ min(latRTKfloat(lowFloat:highFloat)) max(latRTKfloat(lowFloat:highFloat)) ];
  limits2 = [ min(lngRTKfloat(lowFloat:highFloat)) max(lngRTKfloat(lowFloat:highFloat)) ];
  limits3 = [ min(altRTKfloat(lowFloat:highFloat)) max(altRTKfloat(lowFloat:highFloat)) ];
end
if zoomOnSingle == 1
  limits1 = [ min(latSingle(lowSingle:highSingle)) max(latSingle(lowSingle:highSingle)) ];
  limits2 = [ min(lngSingle(lowSingle:highSingle)) max(lngSingle(lowSingle:highSingle)) ];
  limits3 = [ min(altSingle(lowSingle:highSingle)) max(altSingle(lowSingle:highSingle)) ];
end

%prettyfingin 2D-plots
%-------Latitudal--------------------------------------------------------------
title(plot1st, 'Latitudinal Distance', 'interpreter', 'latex')
if printLegends == 1
  leg1 = legend(plot1st, 'RTK fix', 'RTK float', 'Single', 'location', 'southeast');
  set(leg1, 'FontSize', 10,'interpreter', 'latex');
end
xlabel(plot1st, 'Time [h]', 'interpreter', 'latex')
ylabel(plot1st, '$x_\mathrm{n}$ [m]', 'interpreter', 'latex')
grid(plot1st, 'on')
grid(plot1st, 'minor')
if zoomOnSingle || zoomOnFloat || zoomOnFix
  ylim(plot1st, limits1)
  xlim(plot1st, [plotStartTime plotEndTime])
end
%------------------------------------------------------------------------------

%-------Longitudal-------------------------------------------------------------
title(plot2nd, 'Longitudinal Distance', 'interpreter', 'latex')
if printLegends == 1
  leg2 = legend(plot2nd, 'RTK fix', 'RTK float', 'Single', 'location', 'southeast');
  set(leg2, 'FontSize', 10,'interpreter', 'latex');
end
xlabel(plot2nd, 'Time [h]', 'interpreter', 'latex')
ylabel(plot2nd, '$y_\mathrm{n}$ [m]', 'interpreter', 'latex')
grid(plot2nd, 'on')
grid(plot2nd, 'minor')
if zoomOnSingle || zoomOnFloat || zoomOnFix
  ylim(plot2nd, limits2)
  xlim(plot2nd, [plotStartTime plotEndTime])
end
%------------------------------------------------------------------------------

%-------Altitude---------------------------------------------------------------
title(plot3rd, 'Altitude above Mean Sea Level', 'interpreter', 'latex')
if printLegends == 1
  leg3 = legend(plot3rd, 'RTK fix', 'RTK float', 'Single', 'location', 'southeast');
  set(leg3, 'FontSize', 10,'interpreter', 'latex');
end
xlabel(plot3rd, 'Time [h]', 'interpreter', 'latex')
ylabel(plot3rd, 'Altitude [m]', 'interpreter', 'latex')
grid(plot3rd, 'on')
grid(plot3rd, 'minor')
if zoomOnSingle || zoomOnFloat || zoomOnFix
  ylim(plot3rd, limits3)
  xlim(plot3rd, [plotStartTime plotEndTime])
end
%------------------------------------------------------------------------------
