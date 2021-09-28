classdef motor
    %MOTOR Summary of this class goes here
    %   Detailed explanation goes here
    properties(Constant)
        CALIBRATION_POWER = 30
        CALIBRATION_TIMEOUT = 10
        MOTOR_1_DIR = 1
        MOTOR_2_DIR = 0
        COORDINATES_GEAR_A = [0, 0]
        COORDINATES_GEAR_B = [6, 0]
        RADIUS_ARM_1 = 16+1.3125
        RADIUS_ARM_2 = 11
    end
    
    properties
        brickObj
        m1
        m2
        limit1
        limit2
    end
    
    methods
        function obj = motor(brickObj)
            %MOTOR Construct an instance of this class
            %   Detailed explanation goes here
            obj.brickObj = brickObj;
            obj.m1 = brickObj.motorB;
            obj.limit1 = brickObj.sensor1;
            obj.m2 = brickObj.motorC;
            obj.limit2 = brickObj.sensor4;
            
        end
        
        function calibrate(obj)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            
            
            % Drive motor 1 until limit switch gets pushed
            obj.setPower(1, obj.CALIBRATION_POWER);
            obj.m1.start;
            tic;
            while ~obj.readLimit(1) && toc < obj.CALIBRATION_TIMEOUT
            end
            
            if toc >= obj.CALIBRATION_TIMEOUT
                warning("Calibration timeout: Motor 1 failed to calibrate");
            end
            
            obj.m1.stop
            obj.m1.resetTachoCount;
            
            disp("Motor 1 calibrated");
            obj.brickObj.beep;
            
            pause(1);
            
            % Drive motor 2 until limit switch gets pushed
            obj.setPower(2, obj.CALIBRATION_POWER);
            obj.m2.start;
            tic;
            while ~obj.readLimit(2) && toc < obj.CALIBRATION_TIMEOUT
            end
            
            if toc >= obj.CALIBRATION_TIMEOUT
                warning("Calibration timeout: Motor 2 failed to calibrate");
            end
            
            obj.m2.stop
            obj.m2.resetTachoCount;
            
            disp("Motor 2 calibrated");
            obj.brickObj.beep;
            
            
        end        
        
        function isPushed = readLimit(obj, motor)
            % isPushed reads the limit switch of one motor and returns its
            % current state
            
            switch motor
                case 1
                    isPushed = obj.limit1.value;
                case 2
                    isPushed = obj.limit2.value;
            end
        end
        
        function setPower(obj, motor, power)
            % Sets power level of speciefied motor with respect to the
            % turning direction
            
            switch motor
                case 1
                    obj.m1.power = power*(obj.MOTOR_1_DIR * 2-1);
                case 2
                    obj.m2.power = power*(obj.MOTOR_2_DIR * 2-1);
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
            N = N / ( 2 * (x0(2) - x1(2) ));
            
            % 0 = Ax² + Bx + C
            
            A = ( (x0(1) - x1(1) ) / ( x0(2) - x1(2) ) ) * ( ( x0(1) - x1(1) ) / ( x0(2) - x1(2) ) ) + 1;
            B = 2*x0(2) * ( x0(1) - x1(1) ) / ( x0(2) - x1(2) ) - 2*N*( x0(1) - x1(1) ) / ( x0(2) - x1(2)) - 2 * x0(1);
            C = x0(1) * x0(1) + x0(2)*x0(2) + N*N -  R0*R0 - 2*x0(2)*N;
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
            alpha = 180 - 360 * acos( ( IA(1) - motor.COORDINATES_GEAR_A(1) ) / motor.RADIUS_ARM_2 ) / (2*pi);
            beta = 360 * acos( ( IB(1) - motor.COORDINATES_GEAR_B(1) ) / motor.RADIUS_ARM_2 ) / (2*pi);
        end
        
        function pos = angleToPos(alpha)
            % Translate angle of gears to limit position of motors
            pos = ( ( alpha - 14 ) * 2970 / ( 90 -14) );
        end
        function alpha = posToAngle(pos)
            % Translate pos of motors to angle of gears
            alpha = 14 + pos * ( 90 - 14 ) / 2970;
        end
        
        function [limA, limB] = coordinatesToMotorpos(x)
            % Converts coordinate x into motor position
            [alpha, beta] = cartesianToAngles(x);
            limA = angleToPos(alpha);
            limB = angleToPos(beta);
        end
        
        function [E] = anglesToCoordinates(alpha, beta)
            % Converts angles of arms to coordinates.
            
            xC = motor.COORDINATES_GEAR_A(1) - motor.RADIUS_ARM_2 * cos( 2*pi*alpha/360 );
            yC = motor.COORDINATES_GEAR_A(2) + motor.RADIUS_ARM_2 * sin( 2*pi*alpha/360 );
            xD = motor.COORDINATES_GEAR_B(1) + motor.RADIUS_ARM_2 * cos( 2*pi*beta/360 );
            yD = motor.COORDINATES_GEAR_B(2) + motor.RADIUS_ARM_2 * sin( 2*pi*beta/360 );
            
            [E, E2] = motor.getCircIntersect([xC, yC], [xD, yD], motor.RADIUS_ARM_1, motor.RADIUS_ARM_1);
            if E2(2) > E(2)
                E = E2;
            end
        end
        
        function x = motorposToCoordinates(pos1, pos2)
            % Converts motor position into coordinates
            
            alpha = posToAngle(pos1);
            beta = posToAngle(-pos2);
            x = anglesToCoordinates(alpha, beta);
        end
        
        
     
    end
end

