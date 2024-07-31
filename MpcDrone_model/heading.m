function [theta, phi] = heading(t)
    % theta is the angle between x and the projection of t onto the plan
    % (x,y)
    % phi is the angle between y and the projection of t onto the plan
    % (y,z)
    
    theta = zeros(size(t,1),1);
    phi = zeros(size(t,1),1);
    
    for i = 1:size(t,1)

        if (t(i,1)==0)
            if(t(i,2)==0)
                theta(i) = 0;
            elseif(t(i,2)>0)
                theta(i) = pi/2;
            else
                theta(i) = 3*pi/2;
            end
        elseif (t(i,1)>0 && t(i,2)>=0)
            theta(i) = atan(t(i,2)/t(i,1));
        elseif (t(i,1)>0 && t(i,2)<0)
            theta(i) = 2*pi + atan(t(i,2)/t(i,1));
        else
            theta(i) = pi + atan(t(i,2)/t(i,1));
        end

        t_proj = sqrt(t(i,1)^2 + t(i,2)^2);
        
        if (t_proj==0)
            if(t(i,3)==0)
                phi(i) = 0;
            elseif(t(i,3)>0)
                phi(i) = pi/2;
            else
                phi(i) = 3*pi/2;
            end
        elseif (t_proj>0 && t(i,3)>=0)
            phi(i) = atan(t(i,3)/t_proj);
        elseif (t_proj>0 && t(i,3)<0)
            phi(i) = 2*pi + atan(t(i,3)/t_proj);
        else
            phi(i) = pi + atan(t(i,3)/t_proj);
        end
       
    end
    
end



