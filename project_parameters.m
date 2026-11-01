function p = project_parameters()
%PROJECT_PARAMETERS Quarter-car parameters about static equilibrium.
p.sprungMass = 320;
p.unsprungMass = 45;
p.suspensionStiffness = 18000;
p.suspensionDamping = 1200;
p.tireStiffness = 185000;
p.skyhookDamping = 2200;
p.forceLimit = 1200;
p.bumpHeight = 0.02;
p.bumpTime = 0.2;
p.bumpDuration = 0.25;
p.sampleTime = 0.001;
p.stopTime = 3.015;
end
