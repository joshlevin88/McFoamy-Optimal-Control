function output = Endpoint(input)

tf = input.phase.finaltime;

%% Output
output.objective = input.phase.integral + 2*tf;

