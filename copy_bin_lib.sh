#!/bin/bash

#result=${PWD##*/}          # to assign to a variable
ws=${PWD}   

printf 'Top level folder %s\n' "${ws}" # to print to stdout

# symbolic link elas from catkin devel to src/libelas/lib 
mkdir -p "${ws}/src/libelas/lib"
rm -f "${ws}/src/libelas/lib/libelas.so"
ln -s "${ws}/devel/lib/libelas.so"  "${ws}/src/libelas/lib/libelas.so"

mkdir -p "${ws}/src/libelas/bin"
rm -f "${ws}/src/libelas/bin/elas_demo"
ln -s "${ws}/devel/lib/libelas/elas_demo" "${ws}/src/libelas/bin/elas_demo"


# symbolic link libks from catkin devel to src/libks/lib 
mkdir -p "${ws}/src/libks/lib"
rm -f "${ws}/src/libks/lib/libks.so"
ln -s "${ws}/devel/lib/libks.so"   "${ws}/src/libks/lib/libks.so"


mkdir -p "${ws}/src/nodes/capture/lib"
rm -f "${ws}/src/nodes/capture/lib/libcapturenodelet.so"
ln -s "${ws}/devel/lib/libcapturenodelet.so" "${ws}/src/nodes/capture/lib/libcapturenodelet.so"

mkdir -p "${ws}/src/nodes/capture/bin"
rm -f "${ws}/src/nodes/capture/bin/capture"
ln -s "${ws}/devel/lib/capture/capture" "${ws}/src/nodes/capture/bin/capture"


mkdir -p "${ws}/src/nodes/densestereo/lib"
rm -f "${ws}/src/nodes/densestereo/lib/libdense_stereo_nodelet.so"
ln -s "${ws}/devel/lib/libdense_stereo_nodelet.so" "${ws}/src/nodes/densestereo/lib/libdense_stereo_nodelet.so"


mkdir -p "${ws}/src/nodes/occupancymap/lib"
rm -f "${ws}/src/nodes/occupancymap/lib/liboccupancy_map_nodelet.so"
ln -s "${ws}/devel/lib/liboccupancy_map_nodelet.so" "${ws}/src/nodes/occupancymap/lib/liboccupancy_map_nodelet.so"


#/home/ait_jellal/projects/ws_mav/src/nodes/sparsestereo
mkdir -p "${ws}/src/nodes/sparsestereo/lib"
rm -f "${ws}/src/nodes/sparsestereo/lib/libsparse_stereo_nodelet.so"
ln -s "${ws}/devel/lib/libsparse_stereo_nodelet.so" "${ws}/src/nodes/sparsestereo/lib/libsparse_stereo_nodelet.so"

mkdir -p "${ws}/src/nodes/sparsestereo/bin"
rm -f "${ws}/src/nodes/sparsestereo/bin/sparsestereo"
ln -s "${ws}/devel/lib/sparsestereo/sparsestereo" "${ws}/src/nodes/sparsestereo/bin/sparsestereo"

#mkdir -p "${ws}/src/nodes/ds_2_slam/bin"
#rm -f "${ws}/src/nodes/ds_2_slam/bin/ds_2_slam"
#ln -s "${ws}/devel/lib/ds_2_slam/ds_2_slam" "${ws}/src/nodes/ds_2_slam/bin/ds_2_slam"

mkdir -p "${ws}/src/nodes/ds_2_slam/lib"
rm -f "${ws}/src/nodes/ds_2_slam/lib/libds_2_slam_nodelet.so"
ln -s "${ws}/devel/lib/libds_2_slam_nodelet.so" "${ws}/src/nodes/ds_2_slam/lib/libds_2_slam_nodelet.so"

#ptam so
#/home/ait_jellal/projects/ws_mav/src/nodes/sparsestereo
rm -r "${ws}/src/nodes/ptam/lib"
mkdir -p "${ws}/src/nodes/ptam/lib"
ln -s "${ws}/devel/lib//libconversions.so" "${ws}/src/nodes/ptam/lib/libconversions.so"
ln -s "${ws}/devel/lib/libks.so" "${ws}/src/nodes/ptam/lib/libks.so"
ln -s "${ws}/devel/lib/libmapvis.so" "${ws}/src/nodes/ptam/lib/libmapvis.so"
ln -s "${ws}/devel/lib/libnodebase.so" "${ws}/src/nodes/ptam/lib/libnodebase.so"
ln -s "${ws}/devel/lib/libmono_ptam_nodelet.so" "${ws}/src/nodes/ptam/lib/libmono_ptam_nodelet.so"
ln -s "${ws}/devel/lib/libptam.so" "${ws}/src/nodes/ptam/lib/libptam.so"
ln -s "${ws}/devel/lib/librgbd_ptam_nodelet.so" "${ws}/src/nodes/ptam/lib/librgbd_ptam_nodelet.so"
ln -s "${ws}/devel/lib/libstereoptamnodelet.so" "${ws}/src/nodes/ptam/lib/libstereoptamnodelet.so"


