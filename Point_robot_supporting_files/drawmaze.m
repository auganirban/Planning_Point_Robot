function[m1, m2] = drawmaze()
    global maze_height;    global maze_length;    global qs; 
    global maze_thick;     global gap;            global qg;
    global m1; global m2;
    
    x1 = 0:gap:maze_length;
    x2 = gap:gap:maze_length-gap;   
    
    y1 = zeros(size(x1));
    y2 = zeros(size(x2));
    
    for i = 1:length(x1)
        if (floor(x1(i))==x1(i))
            y1(i) = 0;
        else
            y1(i) = maze_height - maze_thick;
        end
    end
    plot (x1,y1,'b')
    hold on
    for i = 1:length(x2)
       if (floor(x2(i))==x2(i))
            y2(i)= maze_thick;            
       else
            y2(i)=maze_height;
       end
    end
    plot (x2,y2,'b')
    m1 = plot(qs(1), qs(2), 'o', 'MarkerFaceColor', 'r');
    m2 = plot(qg(1), qg(2), 'o', 'MarkerFaceColor', 'g');
% % %     legend([m1, m2], {'Start', 'Goal'}, 'Location', 'NorthEast');
    xlabel('x [m]')
    ylabel('y [m]')
    title('Maze')
    grid on
end