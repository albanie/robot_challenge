load('laserscans_demo.mat');
addpath('../pb-parsers/mrg')

close all;
fig1 = figure;
fig2 = figure;
fig3 = figure;

for idx=1:size(scan_log,2)
    scan = scan_log{idx};
    if ~isempty(scan)
        
        set(0, 'currentfigure', fig1);
        subplot(2,1,1);
        plot(scan.ranges);
        ylabel('Range (m)');
        subplot(2,1,2);
        plot(scan.reflectances);
        ylabel('Reflectance');
        drawnow
        
        set(0, 'currentfigure', fig2);
        clf;
        hold on;
        ShowLaserScan(scan);
        [pole_cartesian, pole_polar] = PoleDetector(scan, 500, 0.2);
        
        scatter(pole_cartesian(:,1), pole_cartesian(:,2), 20, 'o', 'red') %, 'filled');
        xlabel('x (m)');
        ylabel('y (m)');
        
        set(0, 'currentfigure', fig3);
        polar(pole_polar(:,1),pole_polar(:,2),'*');
        
        drawnow
    end
end
