#!/usr/bin/env bash
# This script installs the following dependencies:
#
# - Mono JIT
# - csharp
# - MonoDevelop
# - NUnit
# - mysql-server
#
# On 16.04 you will also need:
#
# - mono-reference-assemblies-2.0
# - mono-reference-assemblies-3.5
# - mono-reference-assemblies-4.0
#
# Which installs the frameworks required to create .Net 3.5 dlls and assemblies
# (you need all 3).

# Import the Linux Version Number Variables
. /etc/lsb-release

sudo apt-get update &&  sudo apt-get install -y                                \
  mysql-server                                                                 \
  mono-complete                                                                \
  monodevelop                                                                  \
  monodevelop-nunit                                                            \
  git                                                                          \
  nunit                                                                        \
  nunit-console

if [ "$DISTRIB_RELEASE" = "16.04" ]; then
  sudo apt-get install -y                                                      \
    mono-reference-assemblies-2.0                                              \
    mono-reference-assemblies-3.5                                              \
    mono-reference-assemblies-4.0
fi
