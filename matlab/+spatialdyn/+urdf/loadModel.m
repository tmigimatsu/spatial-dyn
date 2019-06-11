function [ab] = loadModel(path_urdf)
%LoadModel Wrapper for spatial_dyn::urdf::LoadModel()
%   Detailed explanation goes here
if isempty(which('spatial_dyn_load_model'))
    [dir_function,~,~] = fileparts(mfilename('fullpath'));
    dir_lib = dir_function(1:strfind(dir_function, '/+spatialdyn') - 1);
    dir_spatialdyn = dir_function(1:strfind(dir_lib, '/matlab') - 1);
    dir_mex = fullfile(dir_spatialdyn, 'build', 'matlab');
    addpath(dir_mex);
end

ab = spatialdyn.ArticulatedBody(spatial_dyn_load_model(path_urdf));

end