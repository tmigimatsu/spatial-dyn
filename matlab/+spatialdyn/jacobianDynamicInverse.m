function [argout] = jacobianDynamicInverse(varargin)
%jacobianDynamicInverse Wrapper for spatial_dyn::JacobianDynamicInverse()
%   Detailed explanation goes here
if isempty(which('spatial_dyn_jacobian_dynamic_inverse'))
    [dir_function,~,~] = fileparts(mfilename('fullpath'));
    dir_lib = dir_function(1:strfind(dir_function, '/+spatialdyn') - 1);
    dir_spatialdyn = dir_function(1:strfind(dir_lib, '/matlab') - 1);
    dir_mex = fullfile(dir_spatialdyn, 'build', 'matlab');
    addpath(dir_mex);
end

argout = spatial_dyn_jacobian_dynamic_inverse(varargin{:});

end
