if [[ "${BASH_SOURCE[0]}" != "${0}" ]]
then
	echo "Script is being sourced"
else
	echo "ERROR: Script is a subshell"
	echo "To affect your current shell enviroment source this script with:"
	echo "source env.sh"
	exit
fi

# check that this file is run
export ENV_RUN=true

# if acados folder not specified assume parent of the folder of the single examples
ACADOS_INSTALL_DIR=${ACADOS_INSTALL_DIR:-"$(pwd)/../../../external/acados/"}
export ACADOS_INSTALL_DIR
echo
echo "ACADOS_INSTALL_DIR=$ACADOS_INSTALL_DIR"

# export casadi folder and matlab/octave mex folder
# MATLAB case
export MATLABPATH=$MATLABPATH:$ACADOS_INSTALL_DIR/external/casadi-matlab/
export MATLABPATH=$MATLABPATH:$ACADOS_INSTALL_DIR/interfaces/acados_matlab_octave/
export MATLABPATH=$MATLABPATH:$ACADOS_INSTALL_DIR/interfaces/acados_matlab_octave/acados_template_mex/
echo
echo "MATLABPATH=$MATLABPATH"

# export acados mex flags
#export ACADOS_MEX_FLAGS="GCC=/usr/bin/gcc-4.9"

# if model folder not specified assume this folder
MODEL_FOLDER=${MODEL_FOLDER:-"./build"}
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ACADOS_INSTALL_DIR/lib:$MODEL_FOLDER
echo "LD_LIBRARY_PATH=$LD_LIBRARY_PATH"


