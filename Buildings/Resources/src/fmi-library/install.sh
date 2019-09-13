# This script installs the fmi library that is used to link to EnergyPlus.
# This file should be replaced with a cmake file that builds the binaries
# and copies the files as below.
# It also need to be updated to allow binaries for the different
# operating systems.
set -e

FMILIB=~/proj/ldrd/bie/modeling/svn-fmi-library/trunk
# Make sure we are in right directory
CURDIR=`pwd`
DIR=`basename $CURDIR`
echo $DIR
if [ $DIR != "fmi-library" ]; then
    echo "Error: This script needs to be run from the Resources/src/fmi-library"
    exit 1
fi
if [ ! -d $FMILIB_INST ]; then
    echo "Error: Directory $FMILIB_INST does not exist"
    exit 1
fi

# Resource directory
RESDIR=`dirname $CURDIR`
RESDIR=`dirname $RESDIR`

# Compile library
cd $FMILIB
rm -rf install
rm -rf build-fmil
mkdir build-fmil; cd build-fmil
cmake DFMILIB_BUILD_SHARED_LIB=OFF DFMILIB_GENERATE_DOXYGEN_DOC=OFF ..
make install test
cd $CURDIR

# Copy installation package
rm -rf $RESDIR/thirdParty/fmi-library
mkdir -p $RESDIR/thirdParty/fmi-library/
cp -r $FMILIB/install $RESDIR/thirdParty/fmi-library/
# Move .so file
mv $RESDIR/thirdParty/fmi-library/install/lib/libfmilib_shared.so $RESDIR/Library/linux64/
# Delete static library
rm $RESDIR/thirdParty/fmi-library/install/lib/libfmilib.a
rmdir $RESDIR/thirdParty/fmi-library/install/lib

# Move header files so that JModelica can resolve statements
# like #include <FMI/fmi_import_context.h>
rm -f $RESDIR/C-Sources/EnergyPlus/fmilib_config.h
rm -f $RESDIR/C-Sources/EnergyPlus/fmilib.h
rm -rf $RESDIR/C-Sources/EnergyPlus/FMI
rm -rf $RESDIR/C-Sources/EnergyPlus/FMI1
rm -rf $RESDIR/C-Sources/EnergyPlus/FMI2
rm -rf $RESDIR/C-Sources/EnergyPlus/JM
mv $RESDIR/thirdParty/fmi-library/install/include/{FMI,FMI1,FMI2,fmilib_config.h,fmilib.h,JM} $RESDIR/C-Sources/EnergyPlus/
echo "Built $RESDIR/C-Sources/EnergyPlus"
