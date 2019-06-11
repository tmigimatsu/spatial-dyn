classdef ArticulatedBody
    %ArticulatedBody ArticulatedBody wrapper for spatial_dyn
    %   Detailed explanation goes here
    
    properties% (SetAccess = private, Hidden = true)
        ptr_ab;
    end
    
    methods
        function obj = ArticulatedBody(ptr_ab)
            obj.ptr_ab = ptr_ab;
        end
        function delete(obj)
            if isempty(which('spatial_dyn_delete_articulated_body'))
                [dir_function,~,~] = fileparts(mfilename('fullpath'));
                dir_lib = dir_function(1:strfind(dir_function, '/+spatialdyn') - 1);
                dir_spatialdyn = dir_function(1:strfind(dir_lib, '/matlab') - 1);
                dir_mex = fullfile(dir_spatialdyn, 'build', 'matlab');
                addpath(dir_mex);
            end
            spatial_dyn_delete_articulated_body(obj.ptr_ab)
        end
    end
    
end