function [G,F] = gripper(scale,fingers_l,griper_w,thick)

if nargin < 1
    scale=1;
    fingers_l = 4;
    griper_w = 4;
    thick = 0.5;
end

gg = scale*[griper_w/2 0 0
            0 0 0
            -griper_w/2 0 0
            -griper_w/2 fingers_l 0
            -griper_w/2+0.5 fingers_l 0
            -griper_w/2+0.5 0.5 0
            griper_w/2-0.5 0.5 0
            griper_w/2-0.5 fingers_l 0
            griper_w/2 fingers_l 0
        
            griper_w/2 0 thick
            0 0 0.5
            -griper_w/2 0 0.5
            -griper_w/2 fingers_l thick
            -griper_w/2+0.5 fingers_l thick
            -griper_w/2+0.5 0.5 thick
            griper_w/2-0.5 0.5 thick
            griper_w/2-0.5 fingers_l thick
            griper_w/2 fingers_l thick
            ];

% Add homogeneous coordinate to ease calculations
G = [gg'; ones(1,size(gg',2))];

F = [
    1 2 3 4 5 6 7 8 9
    10 11 12 13 14 15 16 17 18
    3 4 13 12 12 12 12 12 12
    12 3 1 10 10 10 10 10 10
    10 1 9 18 18 18 18 18 18
    ];

