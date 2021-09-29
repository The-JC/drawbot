classdef motor
    %MOTOR Summary of this class goes here
    %   Detailed explanation goes here
    properties(Constant)
        CALIBRATION_POWER = 30
        CALIBRATION_TIMEOUT = 25
        MOTOR_1_DIR = 0
        MOTOR_2_DIR = 1
        COORDINATES_GEAR_A = [0, 0]
        COORDINATES_GEAR_B = [6, 0]
        RADIUS_ARM_1 = 16+1.3125
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
        
        function start(obj, motor_label, power, limitValue, speedRegulation)
            arguments
                obj motor;
                motor_label char;
                power double;
                limitValue double {isnumeric} = 0;
                speedRegulation double {isnumeric} = 0;
            end
            % Sets power level of speciefied motor with respect to the
            % turning direction
            
            switch motor_label
                case 'A'
                    obj.mA.limitValue = abs(limitValue);
                    obj.mA.speedRegulation = speedRegulation;
                    obj.mA.power = power;
                    if ~obj.mA.isRunning
                        obj.mA.start;
                    end
                case 'B'
                    obj.mB.limitValue = abs(limitValue);
                    obj.mB.speedRegulation = speedRegulation;
                    obj.mB.power = power;
                    if ~obj.mB.isRunning
                        obj.mB.start;
                    end
            end
        end
        
        function c = setSpeedToCoordinates(obj, x, max_speed ,init, brake)
            arguments
                obj(1,1) motor;
                x(1,2) double;
                max_speed(1,1) double;
                init(1, 2) double = [inf, inf];
                brake(1,1) double = 0;
            end
            
            c=0;
            
            % Get current position of pen
            current_position = [obj.mB.tachoCount, obj.mA.tachoCount];
            current_coordinates = motor.motorposToCoordinates(current_position(1), current_position(2));
            distance = norm(current_coordinates - x);    
            
            % Check if limit is exceded
            if ~init(1)==inf && ~init(2)==inf
                too_far = ( 180 - motor.getAngle(init, x, current_coordinates ) ) >= 90;
            else
                too_far = false;
            end
            if too_far || (distance < 0.1 && brake < 1 ) || distance < 0.05
                return
            end
            
            next = current_coordinates + ( x - current_coordinates ) / ( distance * 100 );
            
            [next_pos_B, next_pos_A] = motor.coordinatesToMotorpos(next);
            distance_position = [next_pos_B - current_position(1), next_pos_A - current_position(2)];
            
            speed = max_speed;
            slow_down_distance = ( max_speed / 20 );
            if distance < slow_down_distance
                speed = speed - (slow_down_distance - distance)/slow_down_distance * (brake * (max_speed-20));
            end
            
            
            if abs(distance_position(1)) > abs(distance_position(2))
                v_speed = [speed, abs( speed / distance_position(2) * distance_position(1) )];
            else
                v_speed = [abs( speed / distance_position(2) * distance_position(1) ), speed];
            end
            
            v_speed(v_speed > 100) = 100;
            
%              disp([" X: ", current_coordinates(1), " Y: ", current_coordinates(2), " TachoCount (B,A)=",current_position," d=", distance, " Speed-B:", v_speed(1) * sign(distance_position(1)), " Speed-A:", v_speed(2) * sign(distance_position(2))]);
            
            obj.start('B', v_speed(1) * sign(distance_position(1)), 0, 1);
            obj.start('A', v_speed(2) * sign(distance_position(2)), 0, 1);
            
            c=1;
        end
        
        function [v_speed] = calculateSpeed(obj, x, max_speed, init, brake)
            arguments
                obj(1,1) motor;
                x(1,2) double;
                max_speed(1,1) double;
                init(1, 2) double = [inf, inf];
                brake(1,1) double = 0;
            end
            
            % Get current position of pen
            current_position = [obj.mB.tachoCount, obj.mA.tachoCount];
            current_coordinates = motor.motorposToCoordinates(current_position(1), current_position(2));
            distance = norm(current_coordinates - x);  
            
            [new_pos_B, new_pos_A] = motor.coordinatesToMotorpos(next);
            distance_position = [new_pos_B - current_position(1), new_pos_A - current_position(2)];
            
            if abs(distance_position(1)) > abs(distance_position(2))
                v_speed = [speed, abs( speed / distance_position(2) * distance_position(1) )];
            else
                v_speed = [abs( speed / distance_position(2) * distance_position(1) ), speed];
            end
            
        end
        
        function gotoPoint(obj, x, brake, last, max_speed)
            arguments
                obj motor;
                x(2, 1) double;
                brake double = 1;
                last(2,1) double = [NaN, NaN];
                max_speed double = 30;
            end
            
            if isnan(last(1)) || isnan(last(2))
                init_pos = [obj.mB.tachoCount, obj.mA.tachoCount];
                init_coord = motor.motorposToCoordinates(init_pos(1), init_pos(2));
            else
                init_coord = last;
            end
            
            max_speed_ = 5;
            
            
            if brake == 1
                obj.mA.brakeMode = 'Brake';
                obj.mB.brakeMode = 'Brake';
                obj.mA.stop;
                obj.mB.stop;
            else
                obj.mA.brakeMode = 'Coast';
                obj.mB.brakeMode = 'Coast';
                obj.mA.stop;
                obj.mB.stop;
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
     
    end
end

