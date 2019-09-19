classdef PIDController
properties 
    previousError;
    errorIntegral;
    output;
end
methods 
    function obj = PIDController()
        obj.previousError = 0;
        obj.errorIntegral = 0;
        output = 0;
    end
    
    function obj = calculatePIDOutput(obj, error, Kp, Ki, Kd, dt)
        obj.errorIntegral = obj.errorIntegral+error*dt;
        deriv = (error-obj.previousError)/dt;
        obj.previousError = error;
        
        obj.output = Kp*error+Ki*obj.errorIntegral+Kd*deriv;
    end
    
    function obj = calculatePIDOutput_withDeriv(obj, error, derivError, Kp, Ki, Kd, dt)
        obj.errorIntegral = obj.errorIntegral+error*dt;
        deriv = derivError;
        
        obj.output = Kp*error+Ki*obj.errorIntegral+Kd*deriv;
    end
    
    function obj = constrainWithAntiWindup(obj, output, maxValue, dt)
        constrainedOutput = output;
        AntiWindUp = 0;
        if abs(output)>maxValue 
            if (output>0)
                AntiWindUp = maxValue-output;
                constrainedOutput = maxValue;
            else
                AntiWindUp = -maxValue+output;
                constrainedOutput = -maxValue;
            end
        end
        obj.errorIntegral = obj.errorIntegral+AntiWindUp*dt;
        obj.output = constrainedOutput;
     end
end
end
