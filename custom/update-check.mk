# Warns at build time when this copy of the code is behind GitHub, so nobody downloads
# old code to the robot by accident. Included from the makefile.
#
# It runs `git fetch` and compares this branch with its GitHub branch. If there's no
# internet, no git, or the branch isn't tracking GitHub, it stays quiet and the build goes on.
#
# Set UPDATE_CHECK_STRICT = 1 below to stop the build instead of just warning.
UPDATE_CHECK_STRICT = 0

ifeq ($(OS),Windows_NT)
UC_NULL = nul
else
UC_NULL = /dev/null
endif

# Run the check only once per build, even if this file gets included twice
ifndef UPDATE_CHECK_LOADED
UPDATE_CHECK_LOADED = 1

# Skip on `make clean`
ifneq ($(MAKECMDGOALS),clean)

UC_GIT = git -c http.lowSpeedLimit=1000 -c http.lowSpeedTime=5

ifneq ($(shell git rev-parse --abbrev-ref --symbolic-full-name @{u} 2>$(UC_NULL)),)
UC_FETCH  := $(shell $(UC_GIT) fetch --quiet 2>$(UC_NULL))
UC_BEHIND := $(shell git rev-list --count HEAD..@{u} 2>$(UC_NULL))
UC_UPSTREAM := $(shell git rev-parse --abbrev-ref --symbolic-full-name @{u} 2>$(UC_NULL))

ifneq ($(UC_BEHIND),)
ifneq ($(UC_BEHIND),0)
UC_MSG = OUTDATED CODE: you are $(UC_BEHIND) commit(s) behind $(UC_UPSTREAM). Run "git pull" (or Source Control > Pull in VS Code) before downloading to the robot.
ifeq ($(UPDATE_CHECK_STRICT),1)
$(error $(UC_MSG))
else
$(info )
$(info ************************************************************************)
$(warning $(UC_MSG))
$(info ************************************************************************)
$(info )
endif
endif
endif
endif

endif

endif
