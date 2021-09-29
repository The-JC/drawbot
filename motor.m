classdef motor < handle
    %MOTOR Summary of this class goes here
    %   Detailed explanation goes here
    properties(Constant)
        CALIBRATION_POWER = 30
        CALIBRATION_TIMEOUT = 25
        PEN_MOTOR_POWER = 20
        PEN_MOTOR_VALUE = 90
        MOTOR_1_DIR = 0
        MOTOR_2_DIR = 1
        COORDINATES_GEAR_A = [0, 0]
        COORDINATES_GEAR_B = [6, 0]
        RADIUS_ARM_1 = 16 +1.3125
        RADIUS_ARM_2 = 11
    end
    
    properties
        brickObj
        mA
        mB
        limit1
        limit2
        mAPos
        mBPos
        penDown
    end
    
    methods
        function obj = motor(brickObj)
            %MOTOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.brickObj = brickObj;
            obj.mA = brickObj.motorB;
            obj.limit1 = brickObj.sensor1;
            obj.mB = brickObj.motorC;
            obj.limit2 = brickObj.sensor4;
            
            obj.mAPos = 0;
            obj.mBPos = 0;
            obj.penDown = 1;
            
        end
        
        function calibrate(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            
            % Drive motor 1 until limit switch gets pushed
            obj.setPower('A', obj.CALIBRATION_POWER);
            obj.mA.start;
            tic;
            while ~obj.readLimit('A') && toc < obj.CALIBRATION_TIMEOUT
            end
            
            if toc >= obj.CALIBRATION_TIMEOUT
                warning("Calibration timeout: Motor A failed to calibrate");
            end
            
            obj.mA.stop
            obj.mA.resetTachoCount;
            obj.mA.internalReset;
            
            disp("Motor A calibrated");
            obj.brickObj.beep;
            
            pause(1);
            
            % Drive motor B until limit switch gets pushed
            obj.setPower('B', obj.CALIBRATION_POWER);
            obj.mB.start;
            tic;
            while ~obj.readLimit('B') && toc < obj.CALIBRATION_TIMEOUT
            end
            
            if toc >= obj.CALIBRATION_TIMEOUT
                warning("Calibration timeout: Motor B failed to calibrate");
            end
            
            obj.mB.stop
            obj.mB.resetTachoCount;
            obj.mB.internalReset;
            
            disp("Motor B calibrated");
            obj.brickObj.beep;
            
            
        end        
        
        function isPushed = readLimit(obj, motor)
            % isPushed reads the limit switch of one motor and returns its
            % current state
            
            switch motor
                case 'A'
                    isPushed = obj.limit1.value;
                case 'B'
                    isPushed = obj.limit2.value;
            end
        end
        
        function setPower(obj, motor, power)
            % Sets power level of speciefied motor with respect to the
            % turning direction
            
            switch motor
                case 'A'
                    obj.mA.power = -power*(obj.MOTOR_1_DIR * 2-1);
                case 'B'
                    obj.mB.power = -power*(obj.MOTOR_2_DIR * 2-1);
            end
        end
        
        function start(obj, motor_label, power, limitValue, speedRegulation, smoothStart, smoothStop)
            arguments
                obj motor;
                motor_label char;
                power double;
                limitValue double {isnumeric} = 0;
                speedRegulation double {isnumeric} = 0;
                smoothStart double {isnumeric} = 0;
                smoothStop double {isnumeric} = 0;
            end
            % Sets power level of speciefied motor with respect to the
            % turning direction
            
            switch motor_label
                case 'A'
                    obj.mA.limitValue = abs(limitValue);
                    obj.mA.speedRegulation = speedRegulation;
                    obj.mA.smoothStart = smoothStart;
                    obj.mA.smoothStop = smoothStop;
                    obj.mA.power = power;
                    if ~obj.mA.isRunning
                        obj.mA.start;
                    end
                case 'B'
                    obj.mB.limitValue = abs(limitValue);
                    obj.mB.speedRegulation = speedRegulation;
                    obj.mB.smoothStart = smoothStart;
                    obj.mB.smoothStop = smoothStop;
                    obj.mB.power = power;
                    if ~obj.mB.isRunning
                        obj.mB.start;
                    end
            end
        end
        
        function obj = lowerPen(obj, lowerPen)
            %LOWERPEN drives the motor responsible for lifting the pen
            
            if lowerPen == obj.penDown
                return
            end
            
            m = obj.brickObj.motorA;
            m.speedRegulation = true;
            m.limitMode = 'Tacho';
            m.brakeMode = 'Brake';
            m.limitValue = motor.PEN_MOTOR_VALUE;
            m.resetTachoCount;

            if(lowerPen)
                obj.penDown = 1;
                m.power = motor.PEN_MOTOR_POWER;
            else
                obj.penDown = 0;
                m.power = -motor.PEN_MOTOR_POWER;
            end

            m.start;
            m.waitFor;

            end
        
        function [v_speed] = calculateSpeed(obj, x, max_speed, brake)
            % calculates the power vector needed to drive both motors to keep the pen on its trajectory
            arguments
                obj(1,1) motor;
                x(1,2) double;
                max_speed(1,1) double;
                brake(1,1) double = 0;
            end  
            
            current_position = [obj.mB.tachoCount, obj.mA.tachoCount];
            [new_pos_B, new_pos_A] = motor.coordinatesToMotorpos(x);
            distance_position = [new_pos_B - current_position(1), new_pos_A - current_position(2)];
             
            nor = norm(double(distance_position));
           
            % Calculate speed proportional to distnace left
            v_speed = max_speed* [abs(double(distance_position(1))/nor), abs(double(distance_position(2))/nor)];
%             if abs(distance_position(1)) > abs(distance_position(2))
%                 v_speed = [speed, abs( speed * distance_position(1) / distance_position(2) )];
%             else
%                 v_speed = [abs( ( speed * distance_position(1)) / distance_position(2) ), speed];
%             end
            
            % Limit the speeds
            v_speed(v_speed > 100) = 100;
            v_speed(v_speed < 10) = 10;
            
        end
        
        function gotoPoint(obj, x, brake, max_speed)
            % Drives the tip from one point to another
            arguments
                obj motor;
                x(2, 1) double;
                brake double = 1;
                max_speed double = 30;
            end
            
            % Get initial position of pen
            initial_position = [obj.mB.tachoCount, obj.mA.tachoCount];
            initial_coordinates = motor.motorposToCoordinates(initial_position(1), initial_position(2));
            distance = norm(initial_coordinates - x);
            
            % component vise diffrence of the initial coordinate to the target coordinate
            d = x' - initial_coordinates;
            
            for index = 0:0.1:1
                
                current_position = [obj.mB.tachoCount, obj.mA.tachoCount];
                current_coordinates = motor.motorposToCoordinates(current_position(1), current_position(2));
                dif = initial_coordinates - current_coordinates;
                
                % Calculate the new motor position to be along the trajectory from the initial point to x
                [new_posB, new_posA] = motor.coordinatesToMotorpos(initial_coordinates + d * index);
                new_posB = new_posB - current_position(1);
                new_posA = new_posA - current_position(2);

                v_speed = obj.calculateSpeed(x, max_speed, brake);
                smooth = 0;
                if  abs(new_posB) > 5
                    obj.mB.brakeMode = 'Brake';
                    obj.start('B', v_speed(1)*sign(new_posB), new_posB, 1, smooth, smooth);
                end
                if abs(new_posA) > 5
                    obj.mA.brakeMode = 'Brake';
                    obj.start('A', v_speed(2)*sign(new_posA), new_posA, 1, smooth, smooth);
                end

%                 if abs(new_posB - current_position(1)) > 25
%                     obj.mB.waitFor;
%                 end
%                 if abs(new_posA - current_position(2)) > 25
%                     obj.mA.waitFor;
%                 end
                
                
                % Wait until both motors finished their movements
                while obj.mB.isRunning || obj.mA.isRunning
%                     current_position = [obj.mB.tachoCount, obj.mA.tachoCount];
%                     current_coordinates = motor.motorposToCoordinates(current_position(1), current_position(2));
%                     dif = x' - current_coordinates
%                     disp([obj.mB.tachoCount, obj.mA.tachoCount, v_speed(1)*sign(new_posB), v_speed(2)*sign(new_posA)])
                end
            end
        end
        
        function followPath(obj, points, max_speed)
            % Lets the pen follow a list of points each row is in the format of (x, y, penDown?)
            arguments
                obj motor;
                points(:,3) double;
                max_speed double = 50;
            end
            
            for i = 1:length(points)
                point = points(i,:);
                
                obj.lowerPen(point(3));
                
                x = point(1:2);
                
                obj.gotoPoint(x, 1, max_speed);
                              
            end         
            
        end
        
        function new_points = fitPath(obj, points)
            % Fits a path to the boundaries of our robot
            [bbox_min, bbox_size] = motor.getBoundingBox(points);
            [left_top, left_bottom] = motor.getCircles(obj.RADIUS_ARM_1 -1, obj.RADIUS_ARM_2, obj.COORDINATES_GEAR_A, obj.COORDINATES_GEAR_B);
            
            min_x = max([left_top(1) - left_top(3), left_bottom(1) - left_bottom(3)]);
            
            best_fit = 10000;
            best_fit_x = 0;
            best_fit_y = 0;
            best_scale = 0;
            % Center of the robot
            mx = (obj.COORDINATES_GEAR_A(1) + obj.COORDINATES_GEAR_B(1)) /2;
            
            for x = min_x:0.5:mx
                y1 = motor.getYCircle(left_top, x)-1;
                y2 = motor.getYCircle(left_bottom, x)-1;
                
                if ~isnan(y1) && ~isnan(y2)
                    if y1 > y2
                        if  abs( ( (mx-x)*2 )/( y1-y2 ) - ( bbox_size(1)/bbox_size(2) ) ) < best_fit
                            best_fit = abs((mx-x)*2)/(y1-y2) - (bbox_size(1)/bbox_size(2));
                            best_fit_x = x;
                            best_fit_y = y2;
                            best_scale = (mx-x)*2 / bbox_size(1);
                        end
                    end
                end
            end
            
            new_points = [[],[],[]];
            for i = 1:length(points)
                new_points(i,1) = (points(i,1)-bbox_min(1))*best_scale + best_fit_x;
                new_points(i,2) = (points(i,2)-bbox_min(2))*best_scale + best_fit_y;
                new_points(i,3) = points(i,3);
            end
            
        end
        
    end
    
    methods (Static)
        function [A,B] = getCircIntersect(x0, x1, R0, R1)      
            % Computes the intersection of 2 circles of centres x0 and x1 and radius resp. R0 and R1.
            
            if x0(2) == x1(2)
                x0(2) =x0(2)+0.1;
            end
            
            N = R1*R1 - R0*R0 - x1(1)*x1(1) + x0(1)*x0(1) - x1(2)*x1(2) + x0(2)*x0(2);
            N = N / double( 2 * (x0(2) - x1(2) ));
            
            % 0 = Ax² + Bx + C
            
            A = ( (x0(1) - x1(1) ) / ( x0(2) - x1(2) ) ) * ( ( x0(1) - x1(1) ) / ( x0(2) - x1(2) ) ) + 1;
            B = 2*x0(2) * ( x0(1) - x1(1) ) / ( x0(2) - x1(2) ) - 2*N*( x0(1) - x1(1) ) / ( x0(2) - x1(2)) - 2 * x0(1);
            C = x0(1)^2 + x0(2)^2 + N^2 -  R0^2 - 2*x0(2)*N;
            delta = sqrt(B*B - 4*A*C);
            xA = (-B + delta) / (2*A);
            xB = (-B - delta) / (2*A);
            yA = N - xA * ( x0(1) - x1(1) ) / ( x0(2) - x1(2) );
            yB = N - xB * ( x0(1) - x1(1) ) / ( x0(2) - x1(2) );
            
            A = [xA, yA];
            B = [xB, yB];
        end
        
        function [alpha, beta] = cartesianToAngles(E)
            % Converts coordinates E to angles of robot arms.
            
            % Calculate the best fitting Intersection point for Arm A and B
            [IA, IA2] = motor.getCircIntersect(E, motor.COORDINATES_GEAR_A, motor.RADIUS_ARM_1, motor.RADIUS_ARM_2);
            if IA(1) > IA2(1)
                IA = IA2;
            end
            
            [IB, IB2] = motor.getCircIntersect(E, motor.COORDINATES_GEAR_B, motor.RADIUS_ARM_1, motor.RADIUS_ARM_2);
            if IB(1) < IB2(1)
                IB = IB2;
            end
            
            % Convert the intersection points into angles
            alpha = 180.0 - acos( ( IA(1) - motor.COORDINATES_GEAR_A(1) ) / motor.RADIUS_ARM_2 ) * 360 / (2*pi);
            beta = acos( ( IB(1) - motor.COORDINATES_GEAR_B(1) ) / motor.RADIUS_ARM_2 ) * 360 / (2*pi);
        end
        
        function pos = angleToPos(alpha)
            % Translate angle of gears to limit position of motors
            pos = ( (alpha - 14) * 2970 / ( 90 -14 ) );
        end
        function alpha = posToAngle(pos)
            % Translate pos of motors to angle of gears
            alpha = 14 + pos * ( 90 - 14 ) / 2970;
        end
        
        function [posB, posA] = coordinatesToMotorpos(x)
            % Converts coordinate x into motor position
            [alpha, beta] = motor.cartesianToAngles(x);
            posB = motor.angleToPos(alpha);
            posA = -motor.angleToPos(beta);
        end
        
        function [E] = anglesToCoordinates(alpha, beta)
            % Converts angles of arms to coordinates.
            
            xC = motor.COORDINATES_GEAR_A(1) - motor.RADIUS_ARM_2 * cos( double( 2 * pi * alpha)/360 );
            yC = motor.COORDINATES_GEAR_A(2) + motor.RADIUS_ARM_2 * sin( double( 2 * pi * alpha)/360 );
            xD = motor.COORDINATES_GEAR_B(1) + motor.RADIUS_ARM_2 * cos( double( 2 * pi * beta)/360 );
            yD = motor.COORDINATES_GEAR_B(2) + motor.RADIUS_ARM_2 * sin( double( 2 * pi * beta)/360 );
            
            [E, E2] = motor.getCircIntersect([xC, yC], [xD, yD], motor.RADIUS_ARM_1, motor.RADIUS_ARM_1);
            if E2(2) > E(2)
                E = E2;
            end
        end
        
        function x = motorposToCoordinates(pos1, pos2)
            % Converts motor position into coordinates
            % pos1 of motor A and pos2 of motor B
            
            alpha = motor.posToAngle(pos1);
            beta = motor.posToAngle(-pos2);
            x = motor.anglesToCoordinates(alpha, beta);
        end
        
        function alpha = getAngle(A, B, C)
            % Get angle between the three points A, B & C
            
            ab = norm(B-A);
            bc = norm(C-B);
            ac = norm(C-A);
            cos_abc = (ab^2 + bc^2 - ac^2) / (2*ab*bc);
            alpha = 180 - ( 360 * acos(cos_abc) / (2*pi) );
        end
        
        function [min_p, size] = getBoundingBox(points)
            % Returns the bounding box of a list of points
            min_x = min(points(:,1));
            max_x = max(points(:,1));
            min_y = min(points(:,2));
            max_y = max(points(:,2));
            
            min_p = [min_x, min_y];
            size = [max_x-min_x, max_y-min_y];
        end
        
        function [x] = quad_solve(a, b, c)
            % Solve a quadratic equatien given by a*x^2 + b*x + c = 0
            d = b^2-4*a*c;
            if d < 0
                % Only imaginery solutions
                x = NaN
                return
            elseif d == 0
                % Ther's only one solution of multiplicity two
                x = (-b + sqrt(d))/(2*a)
            else
                x1 = (-b + sqrt(d))/(2*a);
                x2 = (-b - sqrt(d))/(2*a);
                x = max([x1, x2]);
            end
        end
        
        function x = getYCircle(circle, x)
            xC = circle(1);
            yC = circle(2);
            rC = circle(3);
            a = 1;
            b = -2 * yC;
            c = -2*xC*x + yC^2 - rC^2 + x^2 + xC^2;
            x = motor.quad_solve (a,b,c);
        end
        
        function pos = pointPos(x, d, theta)
            % Calculates the coordinates of a point on a circle with angle theta
            theta_rad = theta * 2*pi / 360;
            pos = [x(1) + d*cos(theta_rad), x(2) + d*sin(theta_rad)];
        end
        
        function [left_top, left_bottom] = getCircles(r1, r2, A, B)
            angle_min = 16;
            left_top = [B(1), B(2), r1+r2];
            pos = motor.pointPos(A, r2, 180-angle_min);
            left_bottom = [pos(1), pos(2), r1];
        end
    end
end

