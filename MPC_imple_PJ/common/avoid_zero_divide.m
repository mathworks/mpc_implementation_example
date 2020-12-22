function out = avoid_zero_divide(in, div_min)

if (in < div_min)
    
    if (in >= 0)
        
        temp = div_min;
        
    elseif (in > -div_min)
        
        temp = -div_min;
        
    else
        
        temp = in;
        
    end
    
else
    
    temp = in;
    
end

out = temp;

end
