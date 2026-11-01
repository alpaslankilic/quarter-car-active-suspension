function height = road_profile(t, p)
%ROAD_PROFILE Raised-cosine bump with continuous height and slope.
phase = (t-p.bumpTime)/p.bumpDuration;
height = zeros(size(t));
onBump = phase >= 0 & phase <= 1;
height(onBump) = p.bumpHeight/2*(1-cos(2*pi*phase(onBump)));
end
