function tests = AeroSmokeTest
tests = functiontests(localfunctions);
end

function state = baselineState(speed)
state = struct( ...
    'speed', speed, ...
    'pitchAngle', 0, ...
    'rideHeight', 0.03, ...
    'vehicleManager', struct( ...
        'airDensity', 1.225, ...
        'wheelbase', 1.558, ...
        'staticFrontWeight', 0.5038));
end

function testWholeCarAeroForces(testCase)
aero = lts.components.Aero.WholeCarAero(-0.6, 0.35, 3.0, 1.2);
state = baselineState(30);
verifyEqual(testCase, aero.computeDownforce(state), ...
    0.5 * 1.225 * 3.0 * 30^2, 'AbsTol', 1e-9);
verifyEqual(testCase, aero.computeDrag(state), ...
    0.5 * 1.225 * 1.2 * 30^2, 'AbsTol', 1e-9);
end

function testPitchSensitivityModulatesDownforce(testCase)
plain = lts.components.Aero.WholeCarAero(-0.6, 0.35, 3.0, 1.2, 0);
sensitive = lts.components.Aero.WholeCarAero(-0.6, 0.35, 3.0, 1.2, 2.0);
state = baselineState(30);
state.pitchAngle = 0.05;
verifyGreaterThan(testCase, ...
    sensitive.computeDownforce(state), plain.computeDownforce(state));
end

function testAeroManagerReturnsAxleLoadStruct(testCase)
% Pin the contract shape consumed by lts.simulation.Simulator: the
% axle-load struct with front/rear downforce and drag resultant terms.
mgr = lts.components.Aero.AeroManager();
mgr = mgr.addComponent( ...
    lts.components.Aero.WholeCarAero(-0.6, 0.35, 3.0, 1.2));
mgr = mgr.addComponent( ...
    lts.components.Aero.WholeCarAero(0.7, 0.35, 1.0, 0.4));
forces = mgr.computeForces(baselineState(25));
required = {'Fz_front', 'Fz_rear', 'F_drag', 'dragHeight', 'dragXPosition'};
for i = 1:numel(required)
    verifyTrue(testCase, isfield(forces, required{i}), ...
        'computeForces must return field %s', required{i});
    verifyTrue(testCase, isfinite(forces.(required{i})), ...
        'computeForces field %s must be finite', required{i});
end
verifyEqual(testCase, forces.Fz_front + forces.Fz_rear, ...
    0.5 * 1.225 * (3.0 + 1.0) * 25^2, 'AbsTol', 1e-9);
end
