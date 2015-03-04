function plot_odom(pdat)

	load(pdat);
	N = size(logged_wheel_data,2);
	dat = cell2mat(logged_wheel_data);
	r_cum = [dat.m_r]';
% 	plot(r_cum)
	l_cum = [dat.m_l]';
	dr = r_cum(2:end) - r_cum(1:end-1);
	dl = l_cum(2:end) - l_cum(1:end-1);

	%alpha = 2 * (dl-dr)/L;
	N = size(dr,1);
	L = 0.55;
	dy = (dr+dl)/2;
	dth= -(dl-dr)/ (2 * L);

	pose = zeros([N+1,3]);
	pose(1,:) = [0 0 0];
	for i=2:N+1
		ddy = dy(i-1);
		ddth= dth(i-1);
        
		pose(i,1) = pose(i-1,1) + ddy*sin(pose(i - 1, 3));
		pose(i,2) = pose(i-1,2) + ddy*cos(ddth);
		pose(i,3) = pose(i-1,3) + ddth;
	end
	plot(pose(:,1), pose(:,2));
end
