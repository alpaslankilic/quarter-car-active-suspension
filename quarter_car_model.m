function [A, Bu, Br, gains] = quarter_car_model(p)
%QUARTER_CAR_MODEL States: body position/velocity, wheel position/velocity.
ms = p.sprungMass;
mu = p.unsprungMass;
ks = p.suspensionStiffness;
cs = p.suspensionDamping;
kt = p.tireStiffness;
A = [0, 1, 0, 0; -ks/ms, -cs/ms, ks/ms, cs/ms; ...
    0, 0, 0, 1; ks/mu, cs/mu, -(ks+kt)/mu, -cs/mu];
Bu = [0; 1/ms; 0; -1/mu];
Br = [0; 0; 0; kt/mu];
gains = zeros(3, 4);
gains(2, 2) = p.skyhookDamping;
% Penalize acceleration, suspension travel, tire displacement and force.
acceleration = A(2, :);
travel = [1, 0, -1, 0];
tire = [0, 0, 1, 0];
accelerationScale = 1.2;
Q = acceleration'*acceleration/accelerationScale^2 ...
    + travel'*travel/0.05^2 + tire'*tire/0.025^2;
R = 1/p.forceLimit^2 + Bu(2)^2/accelerationScale^2;
N = acceleration'*Bu(2)/accelerationScale^2;
gains(3, :) = lqr(A, Bu, Q, R, N);
assert(all(real(eig(A-Bu*gains(3, :))) < 0), 'Unstable LQR design.');
end
