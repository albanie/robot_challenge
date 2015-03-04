addpath('../pb-parsers/mrg')

close all;
fig1 = figure;
fig2 = figure;
fig3 = figure;

laser_channel = 'LMS1xx_14280166_laser2d';

clear mexmoos
mexmoos('init', 'SERVERHOST','192.168.0.11','MOOSNAME','PoleDetectorDemo');
mexmoos('REGISTER',laser_channel,0.0);

    

while true
    mailbox = mexmoos('FETCH');
    scan = GetLaserScans(mailbox, laser_channel, true);
    if ~isempty(scan)
        
        set(0, 'currentfigure', fig1);
        subplot(2,1,1);
        plot(scan.ranges);
        ylabel('Range (m)');
        subplot(2,1,2);
        clf;
        hold on;
        plot(scan.reflectances);
        plot(smooth(scan.reflectances,10));
        ylabel('Reflectance');
        drawnow
        
        set(0, 'currentfigure', fig2);
        clf;
        hold on;
        ShowLaserScan(scan);
        [pole_cartesian, pole_polar] = PoleDetector(scan, 1100, 0);
        scatter(pole_cartesian(:,1), pole_cartesian(:,2), 20, 'o', 'red') %, 'filled');
        xlabel('x (m)');
        ylabel('y (m)');
        
        set(0, 'currentfigure', fig3);
        polar(pole_polar(:,1),pole_polar(:,2),'*');

        drawnow
    end
end
