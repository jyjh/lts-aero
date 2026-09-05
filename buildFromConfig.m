function aero = buildFromConfig(cfg)
% BUILDFROMCONFIG Build the aero model described by cfg.aero
%   aero = lts.components.Aero.buildFromConfig(cfg)
%
%   Without cfg.aero.components this returns a single WholeCarAero
%   resultant (the historic behavior; vehicleManager defaults
%   pitchSensitivityClA to 0). With cfg.aero.components it returns an
%   AeroManager over the split device model:
%
%     components.frontWing — FrontWing (+ heightSensitivity)
%     components.rearWing  — RearWing  (+ heightSensitivity)
%     components.floor     — UnderbodyFloor (+ stallHeight, heightExponent)
%     components.body      — WholeCarAero, the pitch/height-INSENSITIVE
%                            residual (bodywork, wheels, driver) so the
%                            device sensitivities act only on the device
%                            share of the measured load, not on all of it
%
%   The split must reproduce the measured whole-car datum exactly at the
%   nominal attitude (zero pitch, zero ride-height deviation):
%     sum(ClA_i) == cfg.ClA,  sum(CdA_i) == cfg.CdA,
%     sum(ClA_i * x_i) / sum(ClA_i) == cfg.xPosition
%   Violations are config arithmetic errors and fail here with a typed
%   error instead of silently shifting the aero map. Car files satisfy
%   this by computing the body residual (ClA, CdA, xPosition) from the
%   chosen device shares in MATLAB arithmetic.
%
%   See also lts.components.Aero.validateConfig

if isfield(cfg, 'components') && ~isempty(cfg.components)
    c = cfg.components;
    mgr = lts.components.Aero.AeroManager();
    mgr = mgr.addComponent(lts.components.Aero.FrontWing( ...
        c.frontWing.xPosition, c.frontWing.zPosition, ...
        c.frontWing.ClA, c.frontWing.CdA, ...
        c.frontWing.pitchSensitivityClA, c.frontWing.heightSensitivity));
    mgr = mgr.addComponent(lts.components.Aero.RearWing( ...
        c.rearWing.xPosition, c.rearWing.zPosition, ...
        c.rearWing.ClA, c.rearWing.CdA, ...
        c.rearWing.pitchSensitivityClA, c.rearWing.heightSensitivity));
    mgr = mgr.addComponent(lts.components.Aero.UnderbodyFloor( ...
        c.floor.xPosition, c.floor.zPosition, ...
        c.floor.ClA, c.floor.CdA, ...
        c.floor.pitchSensitivityClA, c.floor.stallHeight, ...
        c.floor.heightExponent));
    mgr = mgr.addComponent(lts.components.Aero.WholeCarAero( ...
        c.body.xPosition, c.body.zPosition, c.body.ClA, c.body.CdA, 0));

    localCheckSplitTotals(cfg, c);
    aero = mgr;
    return;
end

aero = lts.components.Aero.WholeCarAero( ...
    cfg.xPosition, cfg.zPosition, cfg.ClA, cfg.CdA, ...
    localFieldOr(cfg, 'pitchSensitivityClA', 0));
end

function value = localFieldOr(s, name, fallback)
% Deliberately not lts kit fieldOr: this repository's kit/ is mounted as
% +lts/+util in its own test sandbox but as +Aero/kit in the
% superproject, so kit calls would only resolve in one of the two
% contexts. VehicleManager.fromConfig applies the same default.
if isfield(s, name) && ~isempty(s.(name))
    value = s.(name);
else
    value = fallback;
end
end

function localCheckSplitTotals(cfg, c)
% The measured whole-car datum is the ground truth (spec-sheet aero map);
% the device split may shape its pitch/height response but must not move
% the reference-point totals.
tol = 1e-6;

clA = c.frontWing.ClA + c.rearWing.ClA + c.floor.ClA + c.body.ClA;
if ~localClose(clA, cfg.ClA, tol)
    error('lts_aero_buildFromConfig:ComponentMismatch', ...
        ['cfg.aero.components device ClA sum (%.6g) does not match ' ...
        'cfg.aero.ClA (%.6g). Compute the body residual from the ' ...
        'chosen device shares.'], clA, cfg.ClA);
end

cdA = c.frontWing.CdA + c.rearWing.CdA + c.floor.CdA + c.body.CdA;
if ~localClose(cdA, cfg.CdA, tol)
    error('lts_aero_buildFromConfig:ComponentMismatch', ...
        ['cfg.aero.components device CdA sum (%.6g) does not match ' ...
        'cfg.aero.CdA (%.6g).'], cdA, cfg.CdA);
end

if clA > 0
    cop = (c.frontWing.ClA * c.frontWing.xPosition ...
        + c.rearWing.ClA * c.rearWing.xPosition ...
        + c.floor.ClA * c.floor.xPosition ...
        + c.body.ClA * c.body.xPosition) / clA;
    if ~localClose(cop, cfg.xPosition, tol)
        error('lts_aero_buildFromConfig:ComponentMismatch', ...
            ['cfg.aero.components device center of pressure (%.6g m) ' ...
            'does not match cfg.aero.xPosition (%.6g m). Derive the ' ...
            'body xPosition from the measured CoP.'], cop, cfg.xPosition);
    end
end
end

function tf = localClose(a, b, relTol)
tf = abs(a - b) <= relTol * max([abs(a), abs(b), eps]);
end
