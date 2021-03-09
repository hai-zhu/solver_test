% add folders
% addpath(genpath(pwd));        % uncomment to add all subfolders
addpath([pwd, '/']);
addpath([pwd, '/external']);
addpath(genpath([pwd, '/matlab']));
% utils
addpath(genpath([pwd, '/utils']));
% IPOPT (YALMIP required)
addpath([pwd, '/external/IPOPT-toolbox']);
addpath([pwd, '/external/IPOPT-toolbox/lib']);
addpath([pwd, '/external/IPOPT-toolbox/bin/linux_3']);
addpath([pwd, '/external/IPOPT-toolbox/bin/linux_4']);
addpath([pwd, '/external/IPOPT-toolbox/bin/linux_5']);
addpath([pwd, '/external/IPOPT-toolbox/bin/windows']);
% YALMIP
addpath([pwd, '/external/YALMIP-master']);
addpath([pwd, '/external/YALMIP-master/extras']);
addpath([pwd, '/external/YALMIP-master/solvers']);
addpath([pwd, '/external/YALMIP-master/modules']);
addpath([pwd, '/external/YALMIP-master/modules/parametric']);
addpath([pwd, '/external/YALMIP-master/modules/moment']);
addpath([pwd, '/external/YALMIP-master/modules/global']);
addpath([pwd, '/external/YALMIP-master/modules/sos']);
addpath([pwd, '/external/YALMIP-master/operators']);
% CasADi
if ispc
    addpath([pwd, '/external/casadi-windows-matlabR2016a-v3.5.1']);
elseif isunix
    addpath([pwd, '/external/casadi-linux-matlabR2014b-v3.5.1']);
else
    warning('Casadi not found for the platform');
end
