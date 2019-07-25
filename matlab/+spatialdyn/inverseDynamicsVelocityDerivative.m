function [argout] = inverseDynamicsVelocityDerivative(varargin)
%inverseDynamicsVelocityDerivative Wrapper for spatial_dyn::InverseDynamicsVelocityDerivative()
%   Detailed explanation goes here
if isempty(which('spatial_dyn_inverse_dynamics_velocity_derivative'))
    [dir_function,~,~] = fileparts(mfilename('fullpath'));
    dir_lib = dir_function(1:strfind(dir_function, '/+spatialdyn') - 1);
    dir_spatialdyn = dir_function(1:strfind(dir_lib, '/matlab') - 1);
    dir_mex = fullfile(dir_spatialdyn, 'build', 'matlab');
    addpath(dir_mex);
end

argout = spatial_dyn_inverse_dynamics_velocity_derivative(varargin{:});

end
