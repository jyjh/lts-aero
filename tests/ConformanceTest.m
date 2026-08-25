function tests = ConformanceTest
% CONFORMANCETEST Pin the contract between this repository and the main
% lts repository (contract items 1-3 of the repository split; see the
% Contracts page of the main repository's documentation).
%
%  1. cfg schema   — lts.components.Aero.validateConfig accepts the
%     canonical cfg.aero and rejects bad input with typed errors.
%  2. Interface    — every concrete aero class subclasses AeroComponent
%     and computeForces returns exactly the simulator-facing struct.
%  3. Telemetry producer fields — the computeForces field names below
%     are what StateLogBuilder logs as F_downforce / F_drag /
%     aeroFz_front / aeroFz_rear. Renaming any of them is a contract
%     change: update this test and the Contracts page in the same PR,
%     then coordinate the main-repository change (see "Changing the
%     contract" there).
tests = functiontests(localfunctions);
end

function cfg = canonicalConfig()
% Mirrors the baseline cfg.aero from the main repository's VehicleConfig.
cfg = struct( ...
    'xPosition', -0.084146, ...
    'zPosition', 0.3, ...
    'ClA', 4.10, ...
    'CdA', 1.60, ...
    'pitchSensitivityClA', 0.0);
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

%% ---- 1. Config schema -------------------------------------------------

function testValidateConfigAcceptsCanonicalConfig(testCase)
verifyTrue(testCase, lts.components.Aero.validateConfig(canonicalConfig()));
end

function testValidateConfigAcceptsMinimalConfig(testCase)
% pitchSensitivityClA is optional (VehicleManager defaults it to 0).
cfg = canonicalConfig();
cfg = rmfield(cfg, 'pitchSensitivityClA');
verifyTrue(testCase, lts.components.Aero.validateConfig(cfg));
end

function testValidateConfigRejectsMissingRequiredField(testCase)
cfg = canonicalConfig();
cfg = rmfield(cfg, 'ClA');
verifyError(testCase, ...
    @() lts.components.Aero.validateConfig(cfg), ...
    'lts_aero_validateConfig:MissingField');
end

function testValidateConfigRejectsNonScalar(testCase)
cfg = canonicalConfig();
cfg.CdA = [1.6 2.0];
verifyError(testCase, ...
    @() lts.components.Aero.validateConfig(cfg), ...
    'lts_aero_validateConfig:InvalidScalar');
end

function testValidateConfigRejectsNegativeCdA(testCase)
cfg = canonicalConfig();
cfg.CdA = -0.5;
verifyError(testCase, ...
    @() lts.components.Aero.validateConfig(cfg), ...
    'lts_aero_validateConfig:OutOfRange');
end

function testValidateConfigRejectsNegativeZPosition(testCase)
cfg = canonicalConfig();
cfg.zPosition = -0.1;
verifyError(testCase, ...
    @() lts.components.Aero.validateConfig(cfg), ...
    'lts_aero_validateConfig:OutOfRange');
end

%% ---- 2. Interface (contract item 1) ------------------------------------

function testConcreteClassesSubclassAeroComponent(testCase)
classes = {'AeroManager', 'WholeCarAero', 'FrontWing', 'RearWing', ...
    'UnderbodyFloor'};
for i = 1:numel(classes)
    mc = meta.class.fromName( ...
        sprintf('lts.components.Aero.%s', classes{i}));
    verifyEqual(testCase, numel(mc), 1, ...
        sprintf('%s must resolve as a class.', classes{i}));
    supers = {mc.SuperclassList.Name};
    verifyTrue(testCase, ...
        any(endsWith(supers, 'AeroComponent')), ...
        sprintf('%s must subclass AeroComponent.', classes{i}));
end
end

%% ---- 3. Telemetry producer fields (contract item 3) --------------------

function testComputeForcesPinsSimulatorFacingFields(testCase)
% Exact field set of the struct the Simulator resolves into the
% F_downforce / F_drag / aeroFz_front / aeroFz_rear channels.
mgr = lts.components.Aero.AeroManager();
mgr = mgr.addComponent( ...
    lts.components.Aero.WholeCarAero(-0.6, 0.35, 3.0, 1.2));
forces = mgr.computeForces(baselineState(25));
verifyEqual(testCase, sort(fieldnames(forces)), sort( ...
    {'Fz_front'; 'Fz_rear'; 'F_drag'; 'dragHeight'; 'dragXPosition'}));
end

function testComponentStateInputsPinned(testCase)
% computeDownforce/computeDrag read these vehicleState fields; they are
% part of the aero-side contract with lts.simulation.Simulator.
component = lts.components.Aero.WholeCarAero(-0.6, 0.35, 3.0, 1.2);
state = baselineState(30);
verifyEqual(testCase, component.computeDownforce(state), ...
    0.5 * 1.225 * 3.0 * 30^2, 'AbsTol', 1e-9);
verifyEqual(testCase, component.computeDrag(state), ...
    0.5 * 1.225 * 1.2 * 30^2, 'AbsTol', 1e-9);
end
