function ssw = w2ss(w)

% input --> 3 by 1 
% output --> 3 by 3 skew symmetric matrix


ssw = [0 -w(3) w(2); w(3) 0 -w(1); -w(2) w(1) 0];
