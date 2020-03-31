classdef force_sensor < dynamicprops

    % only this class methods can view/modify
    properties (SetAccess = public)
        crtk_utils;
    end

    methods

        function self = force_sensor(ros_namespace)
            self.crtk_utils = crtk_utils(self, ros_namespace);
            self.crtk_utils.add_measured_cf();
        end

    end % methods
end % class
