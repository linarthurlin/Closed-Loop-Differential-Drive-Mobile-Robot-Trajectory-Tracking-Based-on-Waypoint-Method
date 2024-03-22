function [Controller,xg,yg,thetag,distance] = pointSelection(Controller,arrived_points,allow_points,ld,x,y,theta)
    global points;
    distance = zeros(points,4);
    for i = arrived_points:allow_points
        d = sqrt((Controller.pathplan(i,1)-x)^2+(Controller.pathplan(i,2)-y)^2);
        if (d <= ld)
            distance(i,1) = d;
            distance(i,2) = Controller.pathplan(i,1);
            distance(i,3) = Controller.pathplan(i,2);
            distance(i,4) = Controller.pathplan(i,3);
        end
    end
    distance = sortrows(distance,1,'descend');
    Controller.inplan = [Controller.inplan, distance(:,1)];
    xg = distance(1,2);
    yg = distance(1,3);
    thetag = distance(1,4);
end