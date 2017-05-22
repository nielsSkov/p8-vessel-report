function LOSpoint = FindLOSpoint(wp_k,wp_k1,vessel_pos,R)
%FindLOSpoint(wp_k,wp_k1,vessel_pos,R) finds the crossing point between the
% line formed by the waypoints (wp_k and wp_k1) and the circle centered in 
% (vessel_position) with a radious (R).

LOSpoint = wp_k1;
LOSpoint1 = [0,0];
LOSpoint2 = [0,0];

[~,distLine] = FindDistWP(wp_k,wp_k1,vessel_pos);

if (distLine < R)
    B = sqrt(R^2-distLine^2);
    if(wp_k(1) == wp_k1(1))
        LOSpoint1(1) = wp_k(1);
        LOSpoint1(2) = vessel_pos(2) + B;
        LOSpoint2(1) = wp_k(1);
        LOSpoint2(2) = vessel_pos(2) - B;
    elseif(wp_k(2) == wp_k1(2))
        LOSpoint1(1) = vessel_pos(1) + B;
        LOSpoint1(2) = wp_k(2);
        LOSpoint2(1) = vessel_pos(1) - B;
        LOSpoint2(2) = wp_k(2);
    else
        slope=(wp_k1(2)-wp_k(2))/(wp_k1(1)-wp_k(1));
        t=wp_k(2)-slope*wp_k(1);
        a=1+slope^2;
        b=2*(slope*t-slope*vessel_pos(2)-vessel_pos(1));
        c=vessel_pos(1)^2 + vessel_pos(2)^2 + t^2 - 2*t*vessel_pos(2)-R^2;
        discriminant = b^2-4*a*c;
        LOSpoint1(1)=(-b+sqrt(discriminant))/(2*a);
        LOSpoint1(2)=slope*(LOSpoint1(1)-wp_k(1))+wp_k(2);
        LOSpoint2(1)=(-b-sqrt(discriminant))/(2*a);
        LOSpoint2(2)=slope*(LOSpoint2(1)-wp_k(1))+wp_k(2);
    end
    
    %Checking which is the best crossing point. If point1 is closer to
    %waypoint k+1 and the point is between waypoints we choose 1, else we
    %choose point 2.
    if (norm(LOSpoint1-wp_k1)<norm(wp_k1-LOSpoint2)) %If crossing 1 is closer
        if ((norm(wp_k-LOSpoint1)<norm(wp_k-wp_k1)) && (norm(wp_k1-LOSpoint1)<norm(wp_k-wp_k1)))
            LOSpoint=LOSpoint1;
        end
    else % If crossing 2 is closer
        if((norm(wp_k-LOSpoint2)<norm(wp_k-wp_k1)) && (norm(wp_k1-LOSpoint2)<norm(wp_k-wp_k1)))
            LOSpoint=LOSpoint2;
        end
    end
end


end

