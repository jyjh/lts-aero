function ok = validateConfig(cfg)
% VALIDATECONFIG Validate the cfg.aero struct consumed by this package.
%   ok = lts.components.Aero.validateConfig(cfg)
%
% Contract item 2 of the repository split (see the Contracts page of the
% main repository's documentation): each component repository owns the
% schema of its cfg sub-struct and validates it at the build boundary,
% so a typo in a car file fails here with a typed error instead of
% surfacing mid-simulation as NaN or a silent unit mistake.
%
% Fields (all SI, all finite real scalars):
%   xPosition           [m]   center of pressure relative to CG (+ forward)
%   zPosition           [m]   nominal height above ground (>= 0)
%   ClA                 [m^2] downforce coefficient * area (>= 0)
%   CdA                 [m^2] drag coefficient * area (>= 0)
%   pitchSensitivityClA [1/rad] optional; ClA change per radian of pitch
%
% Returns logical true on success; otherwise throws with identifier
% lts_aero_validateConfig:<Case> (MissingField | InvalidScalar |
% OutOfRange).

required = {'xPosition', 'zPosition', 'ClA', 'CdA'};
for i = 1:numel(required)
    if ~isfield(cfg, required{i}) || isempty(cfg.(required{i}))
        error('lts_aero_validateConfig:MissingField', ...
            'cfg.aero.%s is required.', required{i});
    end
end

localCheckScalar(cfg, 'xPosition');
localCheckScalar(cfg, 'zPosition');
localCheckScalar(cfg, 'ClA');
localCheckScalar(cfg, 'CdA');
if isfield(cfg, 'pitchSensitivityClA') && ~isempty(cfg.pitchSensitivityClA)
    localCheckScalar(cfg, 'pitchSensitivityClA');
end

if cfg.zPosition < 0
    error('lts_aero_validateConfig:OutOfRange', ...
        'cfg.aero.zPosition=%g m must be >= 0.', cfg.zPosition);
end
if cfg.ClA < 0
    error('lts_aero_validateConfig:OutOfRange', ...
        'cfg.aero.ClA=%g must be >= 0.', cfg.ClA);
end
if cfg.CdA < 0
    error('lts_aero_validateConfig:OutOfRange', ...
        'cfg.aero.CdA=%g must be >= 0.', cfg.CdA);
end

ok = true;
end

function localCheckScalar(cfg, name)
value = cfg.(name);
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ~isfinite(value)
    error('lts_aero_validateConfig:InvalidScalar', ...
        'cfg.aero.%s must be a finite real scalar (got %s).', ...
        name, mat2str(value));
end
end
