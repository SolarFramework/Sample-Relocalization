#!/bin/bash

export XPCF_MODULE_ROOT=~/.remaken/packages/linux-gcc
echo "XPCF_MODULE_ROOT=$XPCF_MODULE_ROOT"

# include dependencies path to ld_library_path
ld_library_path="./"
if [ -f "$PWD/$1_conf.xml" ]; then
	for modulePath in $(grep -o "\$XPCF_MODULE_ROOT.*lib" $1_conf.xml)
	do
	   modulePath=${modulePath/"\$XPCF_MODULE_ROOT"/${XPCF_MODULE_ROOT}}
	   if ! [[ $ld_library_path =~ "$modulePath/x86_64/shared/debug" ]]
	   then
		  ld_library_path=$ld_library_path:$modulePath/x86_64/shared/debug
	   fi 
	done
fi

if [ -f "$PWD/$1_Processing_conf.xml" ]; then
	for modulePath in $(grep -o "\$XPCF_MODULE_ROOT.*lib" $1_Processing_conf.xml)
	do
	   modulePath=${modulePath/"\$XPCF_MODULE_ROOT"/${XPCF_MODULE_ROOT}}
	   if ! [[ $ld_library_path =~ "$modulePath/x86_64/shared/debug" ]]
	   then
		  ld_library_path=$ld_library_path:$modulePath/x86_64/shared/debug
	   fi 
	done
fi

if [ -f "$PWD/$1_Producer_conf.xml" ]; then
	for modulePath in $(grep -o "\$XPCF_MODULE_ROOT.*lib" $1_Producer_conf.xml)
	do
	   modulePath=${modulePath/"\$XPCF_MODULE_ROOT"/${XPCF_MODULE_ROOT}}
	   if ! [[ $ld_library_path =~ "$modulePath/x86_64/shared/debug" ]]
	   then
		  ld_library_path=$ld_library_path:$modulePath/x86_64/shared/debug
	   fi 
	done
fi

if [ -f "$PWD/$1_Viewer_conf.xml" ]; then
	for modulePath in $(grep -o "\$XPCF_MODULE_ROOT.*lib" $1_Viewer_conf.xml)
	do
	   modulePath=${modulePath/"\$XPCF_MODULE_ROOT"/${XPCF_MODULE_ROOT}}
	   if ! [[ $ld_library_path =~ "$modulePath/x86_64/shared/debug" ]]
	   then
		  ld_library_path=$ld_library_path:$modulePath/x86_64/shared/debug
	   fi 
	done
fi


echo "LD_LIBRARY_PATH=$ld_library_path $@"
LD_LIBRARY_PATH=$ld_library_path $@



