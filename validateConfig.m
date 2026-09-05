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
% Optional split device model (cfg.aero.components, all four substructs
% required together; see buildFromConfig):
%   components.frontWing  .xPosition .zPosition .ClA .CdA
%                         .pitchSensitivityClA .heightSensitivity
%   components.rearWing   same fields as frontWing
%   components.floor      like frontWing but .stallHeight (> 0) and
%                         .heightExponent (0, 2] instead of
%                         .heightSensitivity
%   components.body       .xPosition .zPosition .ClA .CdA only (the
%                         pitch/height-insensitive residual)
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

if isfield(cfg, 'components') && ~isempty(cfg.components)
    localCheckComponents(cfg.components);
end

ok = true;
end

function localCheckComponents(c)
requiredSubs = {'frontWing', 'rearWing', 'floor', 'body'};
for i = 1:numel(requiredSubs)
    name = requiredSubs{i};
    if ~isfield(c, name) || ~isstruct(c.(name))
        error('lts_aero_validateConfig:MissingField', ...
            'cfg.aero.components.%s (struct) is required when components is present.', ...
            name);
    end
end

wingFields = {'xPosition', 'zPosition', 'ClA', 'CdA', ...
    'pitchSensitivityClA', 'heightSensitivity'};
localCheckDevice(c, 'frontWing', wingFields);
localCheckDevice(c, 'rearWing', wingFields);
localCheckDevice(c, 'floor', {'xPosition', 'zPosition', 'ClA', 'CdA', ...
    'pitchSensitivityClA', 'stallHeight', 'heightExponent'});
localCheckDevice(c, 'body', {'xPosition', 'zPosition', 'ClA', 'CdA'});

devices = {'frontWing', 'rearWing', 'floor', 'body'};
for i = 1:numel(devices)
    d = c.(devices{i});
    if d.ClA < 0 || d.CdA < 0
        error('lts_aero_validateConfig:OutOfRange', ...
            'cfg.aero.components.%s.ClA/CdA must be >= 0.', devices{i});
    end
    if d.zPosition < 0
        error('lts_aero_validateConfig:OutOfRange', ...
            'cfg.aero.components.%s.zPosition=%g m must be >= 0.', ...
            devices{i}, d.zPosition);
    end
end
if c.floor.stallHeight <= 0
    error('lts_aero_validateConfig:OutOfRange', ...
        'cfg.aero.components.floor.stallHeight=%g m must be > 0.', ...
        c.floor.stallHeight);
end
if c.floor.heightExponent <= 0 || c.floor.heightExponent > 2
    error('lts_aero_validateConfig:OutOfRange', ...
        'cfg.aero.components.floor.heightExponent=%g must be in (0, 2].', ...
        c.floor.heightExponent);
end
if c.frontWing.heightSensitivity < 0 || c.rearWing.heightSensitivity < 0
    error('lts_aero_validateConfig:OutOfRange', ...
        'cfg.aero.components wing heightSensitivity must be >= 0.');
end
end

function localCheckDevice(c, name, fields)
for i = 1:numel(fields)
    f = fields{i};
    if ~isfield(c.(name), f) || isempty(c.(name).(f))
        error('lts_aero_validateConfig:MissingField', ...
            'cfg.aero.components.%s.%s is required.', name, f);
    end
    value = c.(name).(f);
    if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ~isfinite(value)
        error('lts_aero_validateConfig:InvalidScalar', ...
            'cfg.aero.components.%s.%s must be a finite real scalar.', ...
            name, f);
    end
end
end

function localCheckScalar(cfg, name)
value = cfg.(name);
if ~isnumeric(value) || ~isreal(value) || ~isscalar(value) || ~isfinite(value)
    error('lts_aero_validateConfig:InvalidScalar', ...
        'cfg.aero.%s must be a finite real scalar (got %s).', ...
        name, mat2str(value));
end
end
