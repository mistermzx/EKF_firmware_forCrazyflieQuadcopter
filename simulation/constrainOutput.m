function constrainedOutput = constrainOutput(output, value)
    constrainedOutput = output;
    if abs(output)>value 
        if (output>0)
            constrainedOutput = value;
        else
            constrainedOutput = -value;
        end
    end
end