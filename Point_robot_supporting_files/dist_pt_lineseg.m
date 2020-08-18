function [closest_dist, closest_pt]= dist_pt_lineseg(point, line_seg)
    alpha = (point - line_seg(:,1))'*(line_seg(:,2) - line_seg(:,1));
    line_len = norm(line_seg(:,2)-line_seg(:,1));
    alpha = alpha/line_len;
    closest_pt = line_seg(:,1) + alpha*(line_seg(:,2) - line_seg(:,1))/line_len;
    closest_dist = norm(closest_pt - point);
    
    if alpha < 0 || alpha > 1
        distA = norm(point-line_seg(:,1));
        distB = norm(point-line_seg(:,2));
        if distA < distB
            closest_pt = line_seg(:,1);
            closest_dist = distA;
        else
            closest_pt = line_seg(:,2);
            closest_dist = distB;
        end
    end

end