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
        geometry_msgs_Transform;
        geometry_msgs_Twist;
        geometry_msgs_Wrench;
    end

    % these should only be used by these class's methods
    properties (Access = private)
        % joint space
        measured_js_subscriber;
        setpoint_js_subscriber;
        servo_jp_publisher;
        servo_jf_publisher;
        move_jp_publisher;
        % cartesian space
        measured_cp_subscriber;
        measured_cv_subscriber;
        body_measured_cf_subscriber;
        setpoint_cp_subscriber;
        setpoint_cv_subscriber;
        setpoint_cf_subscriber;
        servo_cp_publisher;
        spatial_servo_cf_publisher;
        move_cp_publisher;
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

        function frame = ros_transform_to_frame(frame)
            % convert idiotic ROS message type to homogeneous transforms
            position = trvec2tform([frame.Translation.X, frame.Translation.Y, frame.Translation.Z]);
            orientation = quat2tform([frame.Rotation.W, frame.Rotation.X, frame.Rotation.Y, frame.Rotation.Z]);
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

        function frame_to_ros_transform(frame, transform)
            transform.Translation.X = frame(1, 4);
            transform.Translation.Y = frame(2, 4);
            transform.Translation.Z = frame(3, 4);
            quaternion = tform2quat(frame);
            transform.Rotation.W = quaternion(1);
            transform.Rotation.X = quaternion(2);
            transform.Rotation.Y = quaternion(3);
            transform.Rotation.Z = quaternion(4);
        end

        function vector_to_ros_wrench(vector, wrench)
            wrench.Force.X = vector(1);
            wrench.Force.Y = vector(2);
            wrench.Force.Z = vector(3);
            wrench.Torque.X = vector(4);
            wrench.Torque.Y = vector(5);
            wrench.Torque.Z = vector(6);
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
            self.geometry_msgs_Transform = rosmessage(rostype.geometry_msgs_TransformStamped);
            self.geometry_msgs_Twist = rosmessage(rostype.geometry_msgs_TwistStamped);
            self.geometry_msgs_Wrench = rosmessage(rostype.geometry_msgs_WrenchStamped);
        end

        function delete(self)
            for k = keys(self.active_subscribers)
                subscriber = self.active_subscribers(k{1});
                subscriber.NewMessageFcn = @(a, b, c)[];
                subscriber.delete();
            end
        end


        function [jp, jv, jf, timestamp] = measured_js(self)
            if isempty(self.measured_js_subscriber.LatestMessage)
                warning('measured_js has not received messages yet (topic %s)',...
                        self.measured_js_subscriber.TopicName);
                js = [];
                timestamp = 0.0;
                return;
            end
            jp = self.measured_js_subscriber.LatestMessage.Position;
            jv = self.measured_js_subscriber.LatestMessage.Velocity;
            jf = self.measured_js_subscriber.LatestMessage.Effort;
            timestamp = self.ros_time_to_secs(self.measured_js_subscriber.LatestMessage.Header.Stamp);
        end

        function add_measured_js(self)
            topic = strcat(self.ros_namespace, 'measured_js');
            self.measured_js_subscriber = ...
                rossubscriber(topic, rostype.sensor_msgs_JointState);
            self.class_instance.addprop('measured_js');
            self.class_instance.measured_js = @self.measured_js;
            self.active_subscribers('measured_js') = self.measured_js_subscriber;
        end


        function [jp, jv, jf, timestamp] = setpoint_js(self)
            if isempty(self.setpoint_js_subscriber.LatestMessage)
                warning('setpoint_js has not received messages yet (topic %s)',...
                        self.setpoint_js_subscriber.TopicName);
                js = [];
                timestamp = 0.0;
                return;
            end
            jp = self.setpoint_js_subscriber.LatestMessage.Position;
            jv = self.setpoint_js_subscriber.LatestMessage.Velocity;
            jf = self.setpoint_js_subscriber.LatestMessage.Effort;
            timestamp = self.ros_time_to_secs(self.setpoint_js_subscriber.LatestMessage.Header.Stamp);
        end

        function add_setpoint_js(self)
            topic = strcat(self.ros_namespace, 'setpoint_js');
            self.setpoint_js_subscriber = ...
                rossubscriber(topic, rostype.sensor_msgs_JointState);
            self.class_instance.addprop('setpoint_js');
            self.class_instance.setpoint_js = @self.setpoint_js;
            self.active_subscribers('setpoint_js') = self.setpoint_js_subscriber;
        end


        function servo_jp(self, jp)
            self.sensor_msgs_JointState.Position = jp;
            send(self.servo_jp_publisher, self.sensor_msgs_JointState);
        end

        function add_servo_jp(self)
            topic = strcat(self.ros_namespace, 'servo_jp');
            self.servo_jp_publisher = ...
                rospublisher(topic, rostype.sensor_msgs_JointState);
            self.class_instance.addprop('servo_jp');
            self.class_instance.servo_jp = @self.servo_jp;
        end


        function servo_jf(self, jf)
            self.sensor_msgs_JointState.Effort = jf;
            send(self.servo_jf_publisher, self.sensor_msgs_JointState);
        end

        function add_servo_jf(self)
            topic = strcat(self.ros_namespace, 'servo_jf');
            self.servo_jf_publisher = ...
                rospublisher(topic, rostype.sensor_msgs_JointState);
            self.class_instance.addprop('servo_jf');
            self.class_instance.servo_jf = @self.servo_jf;
        end


        function move_jp(self, jp)
            self.sensor_msgs_JointState.Position = jp;
            send(self.move_jp_publisher, self.sensor_msgs_JointState);
        end

        function add_move_jp(self)
            topic = strcat(self.ros_namespace, 'move_jp');
            self.move_jp_publisher = ...
                rospublisher(topic, rostype.sensor_msgs_JointState);
            self.class_instance.addprop('move_jp');
            self.class_instance.move_jp = @self.move_jp;
        end


        function [cp, timestamp] = measured_cp(self)
            if isempty(self.measured_cp_subscriber.LatestMessage)
                warning('measured_cp has not received messages yet (topic %s)',...
                        self.measured_cp_subscriber.TopicName);
                cp = [];
                timestamp = 0.0;
                return;
            end
            cp = self.ros_transform_to_frame(self.measured_cp_subscriber.LatestMessage.Transform);
            timestamp = self.ros_time_to_secs(self.measured_cp_subscriber.LatestMessage.Header.Stamp);
        end

        function add_measured_cp(self)
            topic = strcat(self.ros_namespace, 'measured_cp');
            self.measured_cp_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_TransformStamped);
            self.class_instance.addprop('measured_cp');
            self.class_instance.measured_cp = @self.measured_cp;
            self.active_subscribers('measured_cp') = self.measured_cp_subscriber;
        end


        function [cv, timestamp] = measured_cv(self)
            if isempty(self.measured_cv_subscriber.LatestMessage)
                warning('measured_cv has not received messages yet (topic %s)',...
                        self.measured_cv_subscriber.TopicName);
                cv = [];
                timestamp = 0.0;
                return;
            end
            cv = self.ros_twist_to_vector(self.measured_cv_subscriber.LatestMessage.Twist);
            timestamp = self.ros_time_to_secs(self.measured_cv_subscriber.LatestMessage.Header.Stamp);
        end

        function add_measured_cv(self)
            topic = strcat(self.ros_namespace, 'measured_cv');
            self.measured_cv_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_TwistStamped);
            self.class_instance.addprop('measured_cv');
            self.class_instance.measured_cv = @self.measured_cv;
            self.active_subscribers('measured_cv') = self.measured_cv_subscriber;
        end


        function [cf, timestamp] = body_measured_cf(self)
            if isempty(self.body_measured_cf_subscriber.LatestMessage)
                warning('body/measured_cf has not received messages yet (topic %s)',...
                        self.body_measured_cf_subscriber.TopicName);
                cf = [];
                timestamp = 0.0;
                return;
            end
            cf = self.ros_wrench_to_vector(self.body_measured_cf_subscriber.LatestMessage.Wrench);
            timestamp = self.ros_time_to_secs(self.body_measured_cf_subscriber.LatestMessage.Header.Stamp);
        end

        function add_body_measured_cf(self)
            topic = strcat(self.ros_namespace, 'body/measured_cf');
            self.body_measured_cf_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_WrenchStamped);
            self.class_instance.addprop('body_measured_cf');
            self.class_instance.body_measured_cf = @self.body_measured_cf;
            self.active_subscribers('body/measured_cf') = self.body_measured_cf_subscriber;
        end


        function [cp, timestamp] = setpoint_cp(self)
            if isempty(self.setpoint_cp_subscriber.LatestMessage)
                warning('setpoint_cp has not received messages yet (topic %s)',...
                        self.setpoint_cp_subscriber.TopicName);
                cp = [];
                timestamp = 0.0;
                return;
            end
            cp = self.ros_transform_to_frame(self.setpoint_cp_subscriber.LatestMessage.Transform);
            timestamp = self.ros_time_to_secs(self.setpoint_cp_subscriber.LatestMessage.Header.Stamp);
        end

        function add_setpoint_cp(self)
            topic = strcat(self.ros_namespace, 'setpoint_cp');
            self.setpoint_cp_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_TransformStamped);
            self.class_instance.addprop('setpoint_cp');
            self.class_instance.setpoint_cp = @self.setpoint_cp;
            self.active_subscribers('setpoint_cp') = self.setpoint_cp_subscriber;
        end


        function [cv, timestamp] = setpoint_cv(self)
            if isempty(self.setpoint_cv_subscriber.LatestMessage)
                warning('setpoint_cv has not received messages yet (topic %s)',...
                        self.setpoint_cv_subscriber.TopicName);
                cv = [];
                timestamp = 0.0;
                return;
            end
            cv = self.ros_twist_to_vector(self.setpoint_cf_subscriber.LatestMessage.Twist);
            timestamp = self.ros_time_to_secs(self.setpoint_cfv_subscriber.LatestMessage.Header.Stamp);
        end

        function add_setpoint_cv(self)
            topic = strcat(self.ros_namespace, 'setpoint_cv');
            self.setpoint_cv_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_Twist);
            self.class_instance.addprop('setpoint_cv');
            self.class_instance.setpoint_cv = @self.setpoint_cv;
            self.active_subscribers('setpoint_cv') = self.setpoint_cv_subscriber;
        end


        function [cf, timestamp] = setpoint_cf(self)
            if isempty(self.setpoint_cf_subscriber.LatestMessage)
                warning('setpoint_cf has not received messages yet (topic %s)',...
                        self.setpoint_cf_subscriber.TopicName);
                cf = [];
                timestamp = 0.0;
                return;
            end
            cf = self.ros_wrench_to_vector(self.setpoint_cf_subscriber.LatestMessage.Wrench);
            timestamp = self.ros_time_to_secs(self.setpoint_cf_subscriber.LatestMessage.Header.Stamp);
        end

        function add_setpoint_cf(self)
            topic = strcat(self.ros_namespace, 'setpoint_cf');
            self.setpoint_cf_subscriber = ...
                rossubscriber(topic, rostype.geometry_msgs_Wrench);
            self.class_instance.addprop('setpoint_cf');
            self.class_instance.setpoint_cf = @self.setpoint_cf;
            self.active_subscribers('setpoint_cf') = self.setpoint_cf_subscriber;
        end


        function servo_cp(self, cp)
            self.frame_to_ros_transform(cp, self.geometry_msgs_Transform.Transform);
            send(self.servo_cp_publisher, self.geometry_msgs_Transform);
        end

        function add_servo_cp(self)
            topic = strcat(self.ros_namespace, 'servo_cp');
            self.servo_cp_publisher = ...
                rospublisher(topic, rostype.geometry_msgs_TransformStamped);
            self.class_instance.addprop('servo_cp');
            self.class_instance.servo_cp = @self.servo_cp;
        end


        function spatial_servo_cf(self, cf)
            self.vector_to_ros_wrench(cf, self.geometry_msgs_Wrench.Wrench);
            send(self.spatial_servo_cf_publisher, self.geometry_msgs_Wrench);
        end

        function add_spatial_servo_cf(self)
            topic = strcat(self.ros_namespace, 'spatial/servo_cf');
            self.spatial_servo_cf_publisher = ...
                rospublisher(topic, rostype.geometry_msgs_WrenchStamped);
            self.class_instance.addprop('spatial_servo_cf');
            self.class_instance.spatial_servo_cf = @self.spatial_servo_cf;
        end


        function move_cp(self, cp)
            self.frame_to_ros_transform(cp, self.geometry_msgs_Transform.Transform);
            send(self.move_cp_publisher, self.geometry_msgs_Transform);
        end

        function add_move_cp(self)
            topic = strcat(self.ros_namespace, 'move_cp');
            self.move_cp_publisher = ...
                rospublisher(topic, rostype.geometry_msgs_TransformStamped);
            self.class_instance.addprop('move_cp');
            self.class_instance.move_cp = @self.move_cp;
        end


    end % methods

end % class
