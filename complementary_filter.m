function [roll_c_n, pitch_c_n] = complementary_filter(fb, wb, g, roll_c, pitch_c, dt)
% complementary_filter

den = fb(1)^2 + fb(2)^2 + fb(3)^2;
pitch_f = fb(1) / den * g(3);
roll_f  = fb(2) / den * g(3);

% pitch_f = atan2(fb(2), fb(3));
% roll_f  = atan2(fb(1), fb(3));

% // Turning around the X axis results in a vector on the Y-axis
%         pitchAcc = atan2f((float)accData[1], (float)accData[2]) * 180 / M_PI;
%         *pitch = *pitch * 0.98 + pitchAcc * 0.02;
%  
% 	// Turning around the Y axis results in a vector on the X-axis
%         rollAcc = atan2f((float)accData[0], (float)accData[2]) * 180 / M_PI;
        
alpha = 0.0005;

roll_c_n  = alpha * roll_f  + (1-alpha) * (dt * wb(1) + roll_c );

pitch_c_n = alpha * pitch_f + (1-alpha) * (dt * wb(2) + pitch_c);

end

