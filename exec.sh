#!/bin/sh
RTM_PATH=/opt/grx/
export PATH=$RTM_PATH/bin/:$PATH
export LD_LIBRARY_PATH=$RTM_PATH/lib:$RTM_PATH/lib/OpenRTM-aist:$LD_LIBRARY_PATH

./CvProcessorComp
