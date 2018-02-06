function [eangles] = rot_to_eangles( R )

    sy = sqrt(R(1,1)^2 + R(2,1)^2);
    
    if(sy > 1e-4)
        phi = atan2(R(3,2), R(3,3));
        psi = atan2(-R(3,1),sy);
        theta = atan2(R(2,1), R(1,1));
    else
        phi = atan2(-R(2,3), R(2,2));
        psi = atan2(-R(3,1), sy);
        theta = 0;
    end
    
    eangles = [phi psi theta]';
end

