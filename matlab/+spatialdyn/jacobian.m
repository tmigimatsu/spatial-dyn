function [argout] = jacobian(varargin)
%jacobian Wrapper for spatial_dyn::Jacobian()
%   Detailed explanation goes here
if isempty(which('spatial_dyn_jacobian'))
    [dir_function,~,~] = fileparts(mfilename('fullpath'));
    dir_lib = dir_function(1:strfind(dir_function, '/+spatialdyn') - 1);
    dir_spatialdyn = dir_function(1:strfind(dir_lib, '/matlab') - 1);
    dir_mex = fullfile(dir_spatialdyn, 'build', 'matlab');
    addpath(dir_mex);
end

argout = spatial_dyn_jacobian(varargin{:});

end
