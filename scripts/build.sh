BASE_DIR=`git rev-parse --show-toplevel`

CLEAN="Build"
SLN=burt-csharp-examples.sln
PROJ=""
ALL=false

usage="\
Builds the solution using monodevelop's commandline tool.

Usage:

./scripts/build.sh --all

Commands:

--clean  \t\t\tClean before performing build. Defaults to building no clean.
--project\tPROJECT_FILE\tSpecify the project file to run. DEFAULT = $PROJ
--all     \t\t\tBuilds all in solution.
-h        \t\t\tDisplays this message.
"

while test $# -gt 0 ; do
  case $1 in
    --clean )
      CLEAN="Clean"
      shift ;;
    --project )
      PROJ=$2
      echo "$PROJ"
      shift ; shift ;;
    --all )
      ALL=true
      shift ;;
    -h )
      echo -e "$usage"; exit 1;;
  esac
done

if [ -z $PROJ ]; then
  ALL=true
fi

if $ALL ; then
  mdtool build --target:$CLEAN $BASE_DIR/$SLN
else
  mdtool build --target:$CLEAN --project:$PROJ $BASE_DIR/$SLN
fi
