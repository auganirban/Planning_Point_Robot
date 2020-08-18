function [contact_normal, dist_c]= get_colli_info(q)
    global ln_segs;         global contact_normal_array; 
    global ln_bias_array;   global mid_pt;
    global safe_dist;
    
    for i = 1:length(contact_normal_array)
        if q(1,1) >= ln_segs(1,1,i) && q(1,1) < ln_segs(1,2,i) 
            ln_bias = ln_bias_array(i);
            contact_normal = contact_normal_array(:,i);
            mid_pt = (ln_segs(:, 2, i) + ln_segs(:, 1, i))/2;
            dist_c = contact_normal'*q + ln_bias;
            break;
        else
            ln_bias = ln_bias_array(end);
            contact_normal = contact_normal_array(:,end);
            mid_pt = (ln_segs(:, 2, end) + ln_segs(:, 1, end))/2;
            dist_c = contact_normal'*q + ln_bias;
        end
    end
end