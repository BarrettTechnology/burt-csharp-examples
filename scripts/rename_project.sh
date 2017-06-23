#!/bin/bash

# Renames a MonoDevelop project that is currently named <old_name> to be named
# <new_name>
# This script must be run from the directory where the project is located.
# All occurances of <old_name> in files in this directory will be replaced, and
# All files in the project folder that contain <old_name> will be renamed.

if [[ $# -ne 2 ]]; then
  echo "Usage: ./rename_project <old_name> <new_name>"
  echo "This script must br run from the directory where the project is located"
  exit 1
fi

OLD_NAME=$(echo $1 | sed 's:/*$::')  # trim trailing /'s
NEW_NAME=$(echo $2 | sed 's:/*$::')
echo "Renaming $OLD_NAME to $NEW_NAME"
mv $OLD_NAME $NEW_NAME
# replace all instance of old_name found in files
grep -rli "$OLD_NAME" * | xargs -i@ sed -i "s/$OLD_NAME/$NEW_NAME/g" @
# rename all files in the project directory to replace old_name with new_name
rename "s/$OLD_NAME/$NEW_NAME/" $NEW_NAME/*

exit 0
