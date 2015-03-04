classdef Car_measurements

	properties
        L = 0.55;
        wheel_readings = [];
        logged_poses = [0 ; 0 ; 0];
	end

	methods
        
        function obj = Car_measurements(initial_d_l, initial_d_r)
			if (nargin > 0) 
				obj.wheel_readings = [initial_d_l; initial_d_r];
			end
		end

        function obj = add_wheel_reading(obj, wheel_odom)
            previous_left = obj.wheel_readings(1, end);
            previous_right = obj.wheel_readings(2, end);
            
            d_l = wheel_odom.m_l-previous_left;
            d_r = wheel_odom.m_r-previous_right;
			
            obj.wheel_readings = [obj.wheel_readings, [d_l ; d_r]];
        end
        
        function last_deltas = get_last_wheel_reading(obj)
            last_deltas = obj.wheel_readings(:, end);
        end
        
        function [estimated_pose] = estimate_pose(obj, d_l, d_r)
            R = (d_l + d_r) / 2 ;
            theta = (d_l - d_r)/ (2* obj.L);
            x = R*cos(theta);
            y = R*sin(theta);
            estimated_pose = [x;y;theta];
        end
        
        function obj = log_pose(obj, estimated_pose)
            obj.logged_poses = [obj.logged_poses, estimated_pose];
        end
    end
end