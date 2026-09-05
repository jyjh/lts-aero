function tests = BuildFromConfigTest
% BUILDFROMCONFIGTEST Pins the cfg.aero component-split build path:
% default WholeCarAero behavior is unchanged, the split model reproduces
% the whole-car datum exactly at the nominal attitude, and its
% pitch/height response moves in the physical directions.
tests = functiontests(localfunctions);
end

function cfg = wholeCarConfig()
cfg = struct( ...
    'xPosition', -0.049202, ...
    'zPosition', 0.3, ...
    'ClA', 4.84, ...
    'CdA', 1.81, ...
    'pitchSensitivityClA', 0.0);
end

function cfg = splitConfig()
% R25-shaped device split: measured whole-car totals preserved by
% construction (body residual computed from the chosen device shares).
cfg = wholeCarConfig();
ClA = cfg.ClA;
CdA = cfg.CdA;
frontWing = struct( ...
    'xPosition', 1.00, 'zPosition', 0.05, ...
    'ClA', 1.36, 'CdA', 0.15, ...
    'pitchSensitivityClA', -1.0, 'heightSensitivity', 0.08);
rearWing = struct( ...
    'xPosition', -1.05, 'zPosition', 0.80, ...
    'ClA', 1.70, 'CdA', 0.60, ...
    'pitchSensitivityClA', 0.6, 'heightSensitivity', 0.02);
floor = struct( ...
    'xPosition', -0.15, 'zPosition', 0.045, ...
    'ClA', 1.07, 'CdA', 0.20, ...
    'pitchSensitivityClA', -0.5, 'stallHeight', 0.012, ...
    'heightExponent', 0.6);
deviceClA = frontWing.ClA + rearWing.ClA + floor.ClA;
bodyClA = ClA - deviceClA;
deviceCdA = frontWing.CdA + rearWing.CdA + floor.CdA;
deviceMoment = frontWing.ClA * frontWing.xPosition ...
    + rearWing.ClA * rearWing.xPosition ...
    + floor.ClA * floor.xPosition;
bodyX = (ClA * cfg.xPosition - deviceMoment) / bodyClA;
cfg.components = struct( ...
    'frontWing', frontWing, ...
    'rearWing', rearWing, ...
    'floor', floor, ...
    'body', struct( ...
        'xPosition', bodyX, 'zPosition', 0.30, ...
        'ClA', bodyClA, 'CdA', CdA - deviceCdA));
end

function state = aeroState(speed, pitchAngle, rideHeight)
state = struct( ...
    'speed', speed, ...
    'pitchAngle', pitchAngle, ...
    'rideHeight', rideHeight, ...
    'vehicleManager', struct( ...
        'airDensity', 1.225, ...
        'wheelbase', 1.528, ...
        'staticFrontWeight', 0.5095));
end

%% ---- Builder dispatch -------------------------------------------------

function testDefaultBuildsWholeCarAero(testCase)
aero = lts.components.Aero.buildFromConfig(wholeCarConfig());
verifyTrue(testCase, isa(aero, 'lts.components.Aero.WholeCarAero'));
end

function testDefaultPitchSensitivityOptional(testCase)
cfg = wholeCarConfig();
cfg = rmfield(cfg, 'pitchSensitivityClA');
aero = lts.components.Aero.buildFromConfig(cfg);
verifyEqual(testCase, aero.pitchSensitivityClA, 0, 'AbsTol', 1e-12);
end

function testComponentSplitBuildsFourDeviceManager(testCase)
aero = lts.components.Aero.buildFromConfig(splitConfig());
verifyTrue(testCase, isa(aero, 'lts.components.Aero.AeroManager'));
verifyEqual(testCase, numel(aero.components), 4);
names = cellfun(@(c) char(c.getName()), aero.components, ...
    'UniformOutput', false);
verifyEqual(testCase, names, ...
    {'Front Wing', 'Rear Wing', 'Underbody Floor', 'Whole Car Aero'});
end

%% ---- Reference-point fidelity -----------------------------------------

function testSplitMatchesWholeCarAtNominalAttitude(testCase)
% The whole-car datum is the measured spec-sheet map: the split must be
% indistinguishable from it at zero pitch / zero ride-height deviation.
cfg = splitConfig();
split = lts.components.Aero.buildFromConfig(cfg);
whole = lts.components.Aero.buildFromConfig(wholeCarConfig());
fSplit = split.computeForces(aeroState(22.22, 0, 0));
fWhole = whole.computeForces(aeroState(22.22, 0, 0));
verifyEqual(testCase, fSplit.Fz_front, fWhole.Fz_front, 'RelTol', 1e-9);
verifyEqual(testCase, fSplit.Fz_rear, fWhole.Fz_rear, 'RelTol', 1e-9);
verifyEqual(testCase, fSplit.F_drag, fWhole.F_drag, 'RelTol', 1e-9);
% dragXPosition is deliberately NOT pinned: the whole-car resultant
% conflates the downforce and drag centers of pressure, while the split
% model weights them per device (the drag center sits where the device
% drag shares put it, not at the measured downforce CoP).
end

%% ---- Response directions ----------------------------------------------

function testNoseUpPitchReducesDownforceAndFrontShare(testCase)
cfg = splitConfig();
aero = lts.components.Aero.buildFromConfig(cfg);
level = aero.computeForces(aeroState(22.22, 0, 0));
noseUp = aero.computeForces(aeroState(22.22, 0.02, 0));
% Braking pitch: wing + floor lose ground effect, rear wing partially
% offsets -> net downforce loss, aero balance moves aft.
verifyLessThan(testCase, noseUp.Fz_front + noseUp.Fz_rear, ...
    level.Fz_front + level.Fz_rear);
frontShareLevel = level.Fz_front / (level.Fz_front + level.Fz_rear);
frontShareNoseUp = noseUp.Fz_front / (noseUp.Fz_front + noseUp.Fz_rear);
verifyLessThan(testCase, frontShareNoseUp, frontShareLevel);
end

function testRideHeightRiseReducesDownforce(testCase)
cfg = splitConfig();
aero = lts.components.Aero.buildFromConfig(cfg);
nominal = aero.computeForces(aeroState(22.22, 0, 0));
risen = aero.computeForces(aeroState(22.22, 0, 0.01));
verifyLessThan(testCase, risen.Fz_front + risen.Fz_rear, ...
    nominal.Fz_front + nominal.Fz_rear);
end

%% ---- Config consistency guards ----------------------------------------

function testSplitTotalsMismatchRejected(testCase)
cfg = splitConfig();
cfg.components.body.ClA = cfg.components.body.ClA + 0.01;
verifyError(testCase, ...
    @() lts.components.Aero.buildFromConfig(cfg), ...
    'lts_aero_buildFromConfig:ComponentMismatch');
end

function testSplitCoPMismatchRejected(testCase)
cfg = splitConfig();
cfg.components.body.xPosition = cfg.components.body.xPosition + 0.05;
verifyError(testCase, ...
    @() lts.components.Aero.buildFromConfig(cfg), ...
    'lts_aero_buildFromConfig:ComponentMismatch');
end

%% ---- Config schema (validateConfig) ------------------------------------

function testValidateConfigAcceptsComponentSplit(testCase)
verifyTrue(testCase, lts.components.Aero.validateConfig(splitConfig()));
end

function testValidateConfigRejectsMissingDeviceField(testCase)
cfg = splitConfig();
cfg.components.floor = rmfield(cfg.components.floor, 'stallHeight');
verifyError(testCase, ...
    @() lts.components.Aero.validateConfig(cfg), ...
    'lts_aero_validateConfig:MissingField');
end

function testValidateConfigRejectsMissingDeviceSubstruct(testCase)
cfg = splitConfig();
cfg = rmfield(cfg.components, 'body');
verifyError(testCase, ...
    @() lts.components.Aero.validateConfig(cfg), ...
    'lts_aero_validateConfig:MissingField');
end

function testValidateConfigRejectsNegativeHeightSensitivity(testCase)
cfg = splitConfig();
cfg.components.frontWing.heightSensitivity = -0.1;
verifyError(testCase, ...
    @() lts.components.Aero.validateConfig(cfg), ...
    'lts_aero_validateConfig:OutOfRange');
end

function testValidateConfigRejectsStallHeightBelowOneMillimeter(testCase)
cfg = splitConfig();
cfg.components.floor.stallHeight = 0;
verifyError(testCase, ...
    @() lts.components.Aero.validateConfig(cfg), ...
    'lts_aero_validateConfig:OutOfRange');
end
