classdef force_sensor < dynamicprops

    % Author(s): Anton Deguet
    %
    % Copyright (c) 2019-2021 Johns Hopkins University, University of Washington, Worcester Polytechnic Institute
    % Released under MIT License

    % only this class methods can view/modify
    properties (Access = protected)
        crtk_utils;
    end

    methods

        function self = force_sensor(ros_namespace)
            self.crtk_utils = crtk_utils(self, ros_namespace);
            self.crtk_utils.add_measured_cf();
        end

    end % methods
end % class
