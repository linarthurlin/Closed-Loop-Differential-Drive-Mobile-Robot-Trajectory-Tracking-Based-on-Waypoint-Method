classdef PIDController < handle
    properties
        cteKp
        cteKi
        cteKd
        cteprevError
        ctesumError
        intergral
        activated
        center
        distance
        
        pathplan
        inplan
        timestamp
        
    end

    methods (Static)
        function obj = Init()
            global points;
            obj.activated = false;
            obj.center = [0 0 0];
            obj.distance = 0;
            obj.pathplan = zeros(points,3);
            obj.inplan = [];
            obj.timestamp = 0;
            
        end

        function obj = PIDCrossTrackError(Kp, Ki, Kd)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            obj.prevError = 0;
            obj.sumError = 0;
            obj.ld = 0;
        end

        function [output, obj] = updateCTE(obj, cte, dt)
            obj.sumError = obj.sumError + cte*dt;
            derivative = (cte - obj.prevError)/dt;
            output = obj.Kp*(cte) + obj.Ki*(obj.sumError) + obj.Kd*(derivative);
            obj.prevError = cte;
        end
    end
end