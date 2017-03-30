function output = Endpoint(input)

%% Start
u0 = input.phase.initialstate(:,1);
w0 = input.phase.initialstate(:,2);

V0 = sqrt(u0^2 + w0^2);

%% Output
output.objective = input.phase.integral;
output.eventgroup.event = V0;


