# rover_interfaces

**Owner(s):** (unassigned)  
**Status:** placeholder — no implementation yet

## Scope

- Custom message, service, and action definitions
- pub/sub interfaces
- server-client interfaces

## Notes

This package is a shared dependency of nearly every other package. Changes here can break
other people's branches silently — call them out in the PR description and announce them to the
team after merge.

## Getting started

Build files (`package.xml`, and `CMakeLists.txt` or `setup.py`) are to be added by the
owner in the first implementation PR — the build type (`ament_cmake` vs `ament_python`)
is the owner's call.

See [CONTRIBUTING.md](../../CONTRIBUTING.md) for the branch and PR workflow.
