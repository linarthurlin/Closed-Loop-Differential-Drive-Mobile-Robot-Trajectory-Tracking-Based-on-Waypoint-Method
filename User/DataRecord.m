classdef DataRecord
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here

    properties
        time
        timestamp
        VuHistory
        TargetDistanceHistory
        OmegaHistory
        nowX
        nowY
        measureX
        measureY
        trueX
        trueY
        error
        radiusICR
        curvature
        ErrorVector
        CrossTrackError
        PoseErrorVector
    end

    methods (Static)
        function obj = Init()
            obj.time = [];
            obj.timestamp = 0;
            obj.VuHistory = [];
            obj.TargetDistanceHistory = [];
            obj.OmegaHistory = [];
            obj.timeX = [];
            obj.timeY = [];
            obj.measureX = [];
            obj.measureY = [];
            obj.trueX = [];
            obj.trueY = [];
            obj.error = [];
            obj.radiusICR = [];
            obj.curvature = [];
            obj.ErrorVector = [];
            obj.CrossTrackError = [];
            obj.PoseErrorVector = [];
        end
    end
end