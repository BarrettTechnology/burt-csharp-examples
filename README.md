# Burt C# Examples
> Example programs for the Burt robot using a CoAP/UDP-based server :robot:

Example programs show how to activate the robot, read command line input, and
command simple movements.

**Table of Contents**:
<!-- TOC depthFrom:2 depthTo:6 withLinks:1 updateOnSave:1 orderedList:0 -->

- [Prerequisites](#prerequisites)
- [Building](#building)
	- [Project MonoDevelop Configurations](#project-monodevelop-configurations)
	- [Building from MonoDevelop](#building-from-monodevelop)
- [Usage (Console Application)](#usage-console-application)
	- [Running an Example from MonoDevelop](#running-an-example-from-monodevelop)
	- [Adding Packages (optional)](#adding-packages-optional)

<!-- /TOC -->

## Prerequisites

If you are using the console provided by Barrett, the prerequisites are already
installed and you can skip this section.

To install the dependencies on Ubuntu 14.04 or 16.04:

```bash
$ ./scripts/install-dependencies.sh
```

To see the specific dependencies/packages please look at `scripts/install-dependencies.sh`

## Building

### Project MonoDevelop Configurations
- Launch MonoDevelop application or open project directly by double-clicking the `.csproj` project file: `File > Open > <project-directory>.csproj`
- Attempt building the project: `Build > Build All (F8)`
  - if the build succeeds without errors, continue with `Usage (Console Application)` below.
  - if a build error exists:
      - Open Project Options
        - `Project > <project-name> Options`
          - `Build -> General -> target Framework -> Mono / .Net 3.5` or
          - `Build -> General -> target Framework -> Mono / .Net 4.0`
      - Add Missing References
        - Click References -> Edit References
          - Find the following system references using the Search Bar and click "Add" to add the selected references to the project.
            - `System.Data`
            - `System.Xml.*`, e.g. `System.Xml`, `System.Xml.Linq`
          - Open .Net Assemblies Tab
            - Browse to `the-project-directory/dll`
              - Add all `.dll` files
          - Click OK to close the dialog

### Building from MonoDevelop

Build -> Build ALL `F8`

## Usage

### Running an Example from MonoDevelop

Right click on the project folder and click `Run`

### Adding Packages (optional)

- open mono project, double click on *.csproj file
- Right-click references folder
- Search for System.Data
- Add, close Dialog
