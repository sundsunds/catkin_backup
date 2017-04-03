#!/bin/bash
cd ~
mkdir -p toon_gvar_libcvd
cd toon_gvar_libcvd

echo pwd


echo " Now pulling TooN..."
git clone git://github.com/edrosten/TooN.git
echo " TooN done for good!"

echo " Now pulling libcvd..."
git clone git://github.com/edrosten/libcvd.git
echo " libcvd done for good!"

echo " Now pulling gvars3..."
git clone git://github.com/edrosten/gvars.git
echo " gvars3 done for good!"

echo " All done."
