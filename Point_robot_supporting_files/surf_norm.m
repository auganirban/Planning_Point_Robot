function [ln_segs, contact_normal_array, ln_bias_array] = surf_norm()
    global maze_height;    global maze_length;    % global mid_pt;
    global maze_thick;     global gap;
    
%     xsegs = gap:gap:maze_length;
    xsegs = 0:gap:maze_length;
    yval_pair1 = [maze_height, maze_thick];
    yval_pair2 = [0, maze_height - maze_thick];
    
    ln_segs = zeros(2, 2, length(xsegs)-1);
    contact_normal_array = [];
    ln_bias_array = [];
    for i = 1:length(xsegs)-1
        if mod(i, 2) ~= 0
            ysegs = yval_pair2;
        else
            ysegs = yval_pair1;
        end
        if i < length(xsegs)-1
            ln_segs(:, :, i) = [xsegs(i), xsegs(i+1); ysegs];
            temp = (ln_segs(:, 2, i) - ln_segs(:, 1, i))/norm(ln_segs(:, 2, i) - ln_segs(:, 1, i));
            ln_bias = ln_segs(1,1,i)*temp(2) - ln_segs(2,1,i)*temp(1);
            contact_normal = [-temp(2); temp(1)];
%             mid_pt = (ln_segs(:, 2, i) + ln_segs(:, 1, i))/2;   
            if mod(i,2) == 0
                contact_normal = -contact_normal;
                ln_bias = -ln_bias;
            end
            contact_normal_array = [contact_normal_array, contact_normal];
            ln_bias_array = [ln_bias_array, ln_bias];
        else
            ln_segs(:, :, i) = [xsegs(i), xsegs(i+1); maze_height-maze_thick, 0];
            temp = (ln_segs(:, 2, i) - ln_segs(:, 1, i))/norm(ln_segs(:, 2, i) - ln_segs(:, 1, i));
            ln_bias = ln_segs(1,1,i)*temp(2) - ln_segs(2,1,i)*temp(1);
            contact_normal = [-temp(2); temp(1)];
%             mid_pt = (ln_segs(:, 2, i) + ln_segs(:, 1, i))/2;
            contact_normal_array = [contact_normal_array, contact_normal];
            ln_bias_array = [ln_bias_array, ln_bias];
        end
    end
end