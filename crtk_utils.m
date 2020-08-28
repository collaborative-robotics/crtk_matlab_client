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
        % operating state
        operating_state_timer;
        operating_state_subscriber;
        move_is_waiting;
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
            % operating state subscriber
            self.operating_state_subscriber = ...
                rossubscriber(self.ros_topic('operating_state'), 'crtk_msgs/operating_state');
            self.operating_state_subscriber.NewMessageFcn = @self.operating_state_callback;
            self.active_subscribers('operating_state') = self.operating_state_subscriber;
            % timer used for operating state
            self.operating_state_timer = timer('ExecutionMode', 'singleShot', ...
                                               'Name', strcat(self.ros_namespace, '_operating_state'), ...
                                               'ObjectVisibility', 'off', ...
                                               'StartDelay', 300.0); % 5 minutes is long enough for any task
            self.operating_state_timer.TimerFcn = { @self.operating_state_timeout };
            % move command not issued
            self.move_is_waiting = true;
        end

        function delete(self)
            % members always created
            delete(self.operating_state_timer);
            delete(self.operating_state_subscriber);
            % delete subscribers that might have been created
            for k = keys(self.active_subscribers)
                subscriber = self.active_subscribers(k{1});
                delete(subscriber);
            end
        end

        function full_topic = ros_topic(self, topic)
            if strcmp(self.ros_namespace, '')
                full_topic = topic;
                return;
            end
            if strcmp(self.ros_namespace(end), '/')
                full_topic = strcat(self.ros_namespace, topic);
                return;
            end
            full_topic = strcat(self.ros_namespace, '/', topic);
        end

        function check_input_is_frame(self, frame)
            if ~isreal(frame)
                error('%s: input must be an array or real numbers, not %s', self.ros_namespace, class(frame));
            end
            if ~ismatrix(frame)
                error('%s: input must be a matrix', self.ros_namespace);
            end
            [nb_rows, nb_cols] = size(frame);
            if (nb_rows ~= 4) || (nb_cols ~=4)
                error('%s: input must be a 4x4 matrix, got %dx%d', self.ros_namespace, nb_rows, nb_cols);
            end
        end

        function operating_state_timeout(self, ~, ~) % second parameter is timer, third is this function
            fprintf('%s: timeout for operating state\n', self.ros_namespace);
        end

        function pre_move(self, timeout_s)
            if (timeout_s > 0.0)
                self.move_is_waiting = true;
                start(self.operating_state_timer);
            end
        end

        function post_move(self, timeout_s)
            if timeout_s > 0.0
                wait(self.operating_state_timer);
            end
        end

        function operating_state_callback(self, ~, message)
            if self.move_is_waiting
                if ~strcmp(message.State, 'ENABLED') || ~message.IsBusy
                    stop(self.operating_state_timer);
                    self.move_is_waiting = false;
                end
            end
            % fprintf('%s is in state %s\n', self.ros_namespace, message.State);
        end

        function [jp, jv, jf, timestamp] = measured_js(self)
            if isempty(self.measured_js_subscriber.LatestMessage)
                warning('measured_js has not received messages yet (topic %s)',...
                        self.measured_js_subscriber.TopicName);
                jp = [];
                jv = [];
                jf = [];
                timestamp = 0.0;
                return;
            end
            jp = self.measured_js_subscriber.LatestMessage.Position;
            jv = self.measured_js_subscriber.LatestMessage.Velocity;
            jf = self.measured_js_subscriber.LatestMessage.Effort;
            timestamp = self.ros_time_to_secs(self.measured_js_subscriber.LatestMessage.Header.Stamp);
        end

        function add_measured_js(self)
            cmd = 'measured_js';
            self.measured_js_subscriber = ...
                rossubscriber(self.ros_topic(cmd), rostype.sensor_msgs_JointState);
            self.class_instance.addprop(cmd);
            self.class_instance.measured_js = @self.measured_js;
            self.active_subscribers(cmd) = self.measured_js_subscriber;
        end


        function [jp, jv, jf, timestamp] = setpoint_js(self)
            if isempty(self.setpoint_js_subscriber.LatestMessage)
                warning('setpoint_js has not received messages yet (topic %s)',...
                        self.setpoint_js_subscriber.TopicName);
                jp = [];
                jv = [];
                jf = [];
                timestamp = 0.0;
                return;
            end
            jp = self.setpoint_js_subscriber.LatestMessage.Position;
            jv = self.setpoint_js_subscriber.LatestMessage.Velocity;
            jf = self.setpoint_js_subscriber.LatestMessage.Effort;
            timestamp = self.ros_time_to_secs(self.setpoint_js_subscriber.LatestMessage.Header.Stamp);
        end

        function add_setpoint_js(self)
            cmd = 'setpoint_js';
            self.setpoint_js_subscriber = ...
                rossubscriber(self.ros_topic(cmd), rostype.sensor_msgs_JointState);
            self.class_instance.addprop(cmd);
            self.class_instance.setpoint_js = @self.setpoint_js;
            self.active_subscribers(cmd) = self.setpoint_js_subscriber;
        end


        function servo_jp(self, jp)
            self.sensor_msgs_JointState.Position = jp;
            send(self.servo_jp_publisher, self.sensor_msgs_JointState);
        end

        function add_servo_jp(self)
            cmd = 'servo_jp';
            self.servo_jp_publisher = ...
                rospublisher(self.ros_topic(cmd), rostype.sensor_msgs_JointState);
            self.class_instance.addprop(cmd);
            self.class_instance.servo_jp = @self.servo_jp;
        end


        function servo_jf(self, jf)
            self.sensor_msgs_JointState.Effort = jf;
            send(self.servo_jf_publisher, self.sensor_msgs_JointState);
        end

        function add_servo_jf(self)
            cmd = 'servo_jf';
            self.servo_jf_publisher = ...
                rospublisher(self.ros_topic(cmd), rostype.sensor_msgs_JointState);
            self.class_instance.addprop(cmd);
            self.class_instance.servo_jf = @self.servo_jf;
        end


        function move_jp(self, jp, timeout_s)
            % default based on number of arguments
            if nargin == 2
                timeout_s = 0.0;
            else
                if timeout_s < 0.0
                    timeout_s = 30.0;
                end
            end
            self.sensor_msgs_JointState.Position = jp;
            self.pre_move(timeout_s);
            send(self.move_jp_publisher, self.sensor_msgs_JointState);
            self.post_move(timeout_s);
        end

        function add_move_jp(self)
            cmd = 'move_jp';
            self.move_jp_publisher = ...
                rospublisher(self.ros_topic(cmd), rostype.sensor_msgs_JointState);
            self.class_instance.addprop(cmd);
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
            cmd = 'measured_cp';
            self.measured_cp_subscriber = ...
                rossubscriber(self.ros_topic(cmd), rostype.geometry_msgs_TransformStamped);
            self.class_instance.addprop(cmd);
            self.class_instance.measured_cp = @self.measured_cp;
            self.active_subscribers(cmd) = self.measured_cp_subscriber;
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
            cmd = 'measured_cv';
            self.measured_cv_subscriber = ...
                rossubscriber(self.ros_topic(cmd), rostype.geometry_msgs_TwistStamped);
            self.class_instance.addprop(cmd);
            self.class_instance.measured_cv = @self.measured_cv;
            self.active_subscribers(cmd) = self.measured_cv_subscriber;
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
            cmd = 'body/measured_cf';
            self.body_measured_cf_subscriber = ...
                rossubscriber(self.ros_topic(cmd), rostype.geometry_msgs_WrenchStamped);
            self.class_instance.addprop('body_measured_cf');
            self.class_instance.body_measured_cf = @self.body_measured_cf;
            self.active_subscribers(cmd) = self.body_measured_cf_subscriber;
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
            cmd = 'setpoint_cp';
            self.setpoint_cp_subscriber = ...
                rossubscriber(self.ros_topic(cmd), rostype.geometry_msgs_TransformStamped);
            self.class_instance.addprop(cmd);
            self.class_instance.setpoint_cp = @self.setpoint_cp;
            self.active_subscribers(cmd) = self.setpoint_cp_subscriber;
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
            timestamp = self.ros_time_to_secs(self.setpoint_cv_subscriber.LatestMessage.Header.Stamp);
        end

        function add_setpoint_cv(self)
            cmd = 'setpoint_cv';
            self.setpoint_cv_subscriber = ...
                rossubscriber(self.ros_topic(cmd), rostype.geometry_msgs_Twist);
            self.class_instance.addprop(cmd);
            self.class_instance.setpoint_cv = @self.setpoint_cv;
            self.active_subscribers(cmd) = self.setpoint_cv_subscriber;
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
            cmd = 'setpoint_cf';
            self.setpoint_cf_subscriber = ...
                rossubscriber(self.ros_topic(cmd), rostype.geometry_msgs_Wrench);
            self.class_instance.addprop(cmd);
            self.class_instance.setpoint_cf = @self.setpoint_cf;
            self.active_subscribers(cmd) = self.setpoint_cf_subscriber;
        end


        function servo_cp(self, cp)
            self.frame_to_ros_transform(cp, self.geometry_msgs_Transform.Transform);
            send(self.servo_cp_publisher, self.geometry_msgs_Transform);
        end

        function add_servo_cp(self)
            cmd = 'servo_cp';
            self.servo_cp_publisher = ...
                rospublisher(self.ros_topic(cmd), rostype.geometry_msgs_TransformStamped);
            self.class_instance.addprop(cmd);
            self.class_instance.servo_cp = @self.servo_cp;
        end


        function spatial_servo_cf(self, cf)
            self.vector_to_ros_wrench(cf, self.geometry_msgs_Wrench.Wrench);
            send(self.spatial_servo_cf_publisher, self.geometry_msgs_Wrench);
        end

        function add_spatial_servo_cf(self)
            cmd = 'spatial/servo_cf';
            self.spatial_servo_cf_publisher = ...
                rospublisher(self.ros_topic(cmd), rostype.geometry_msgs_WrenchStamped);
            self.class_instance.addprop('spatial_servo_cf');
            self.class_instance.spatial_servo_cf = @self.spatial_servo_cf;
        end


        function move_cp(self, cp, timeout_s)
            % default based on number of arguments
            if nargin == 2
                timeout_s = 0.0;
            else
                if timeout_s < 0.0
                    timeout_s = 30.0;
                end
            end
            self.check_input_is_frame(cp);
            self.frame_to_ros_transform(cp, self.geometry_msgs_Transform.Transform);
            self.pre_move(timeout_s);
            send(self.move_cp_publisher, self.geometry_msgs_Transform);
            self.post_move(timeout_s);
        end

        function add_move_cp(self)
            cmd = 'move_cp';
            self.move_cp_publisher = ...
                rospublisher(self.ros_topic(cmd), rostype.geometry_msgs_TransformStamped);
            self.class_instance.addprop(cmd);
            self.class_instance.move_cp = @self.move_cp;
        end


    end % methods

end % class
