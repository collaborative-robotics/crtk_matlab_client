classdef crtk_utils < handle

    % settings that are not supposed to change after constructor
    properties (SetAccess = immutable)
        ros_namespace; % namespace for this arm, should contain head/tail / (default is empty)
        class_instance;
    end

    % so derived class can extend this
    properties (Access = protected)
        % map of active subscribers, used to stop all active subscribers
        active_subscribers = containers.Map();
        % ros messages instances to avoid runtime dynamic creation
        % these must be created in the constructor for crtk_utils
        std_msgs_Bool;
        std_msgs_String;
        sensor_msgs_JointState;
        geometry_msgs_Pose;
        geometry_msgs_Twist;
        geometry_msgs_Wrench;
    end

    % these should only be used by these class's methods
    properties (Access = private)
        % subscriber
        measured_js_subscriber;
        measured_cp_subscriber;
        measured_cv_subscriber;
        measured_cf_subscriber;
        setpoint_jp_subscriber;
        setpoint_jv_subscriber;
        setpoint_jf_subscriber;
        setpoint_cp_subscriber;
        setpoint_cv_subscriber;
        setpoint_cf_subscriber;
    end


    methods(Static)

        function seconds = ros_time_to_secs(stamp)
            % Convert awkward rostime into a single double
            seconds = double(stamp.Sec) + double(stamp.Nsec) * 10^-9;
        end

        function frame = ros_pose_to_frame(pose)
           % convert idiotic ROS message type to homogeneous transforms
           position = trvec2tform([pose.Position.X, pose.Position.Y, pose.Position.Z]);
           orientation = quat2tform([pose.Orientation.W, pose.Orientation.X, pose.Orientation.Y, pose.Orientation.Z]);
           frame = position * orientation;
        end

        function vector = ros_twist_to_vector(twist)
            vector = [twist.Linear.X,  twist.Linear.Y,  twist.Linear.Z, ...
                      twist.Angular.X, twist.Angular.Y, twist.Angular.Z];
        end

        function vector = ros_wrench_to_vector(wrench)
           % convert idiotic ROS message type to a single vector
           vector = [wrench.Force.X,  wrench.Force.Y,  wrench.Force.Z, ...
                     wrench.Torque.X, wrench.Torque.Y, wrench.Torque.Z];
        end

    end % methods(Static)


    methods

        function self = crtk_utils(class_instance, namespace)
            self.class_instance = class_instance;
            self.ros_namespace = namespace;
            % one time creation of messages to prevent lookup and creation at each call
            self.std_msgs_Bool = rosmessage(rostype.std_msgs_Bool);
            self.std_msgs_String = rosmessage(rostype.std_msgs_String);
            self.sensor_msgs_JointState = rosmessage(rostype.sensor_msgs_JointState);
            self.geometry_msgs_Pose = rosmessage(rostype.geometry_msgs_Pose);
            self.geometry_msgs_Twist = rosmessage(rostype.geometry_msgs_Twist);
            self.geometry_msgs_Wrench = rosmessage(rostype.geometry_msgs_Wrench);
        end

        function delete(self)
            for k = keys(self.active_subscribers)
                subscriber = self.active_subscribers(k{1});
                subscriber.NewMessageFcn = @(a, b, c)[];
                subscriber.delete();
            end
        end

        function [cf, timestamp] = measured_cf(self)
            if isempty(self.measured_cf_subscriber.LatestMessage)
                warning('measured_cf has not received messages yet (topic %s)',...
                        self.measured_cf_subscriber.TopicName);
                cf = [];
                timestamp = 0.0;    
                return;
            end
            cf = self.ros_wrench_to_vector(self.measured_cf_subscriber.LatestMessage.Wrench);
            timestamp = self.ros_time_to_secs(self.measured_cf_subscriber.LatestMessage.Header.Stamp);
        end

        function add_measured_cf(self)
            % wrench cartesian current
            topic = strcat(self.ros_namespace, 'measured_cf');
            self.measured_cf_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_WrenchStamped);
            % add property to user class
            self.class_instance.addprop('measured_cf');
            self.class_instance.measured_cf = @self.measured_cf;
            self.active_subscribers('measured_cf') = self.measured_cf_subscriber;
        end

    end % methods

end % class
