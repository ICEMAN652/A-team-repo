# Warns at build time when this copy of the code is behind GitHub, so nobody downloads
# old code to the robot by accident. Included from the makefile.
#
# It runs `git fetch` and compares this branch with its GitHub branch. If there's no
# internet, no git, or the branch isn't tracking GitHub, it stays quiet and the build goes on.
#
# Set UPDATE_CHECK_STRICT = 1 below to stop the build instead of just warning.
# Strict mode keeps the Build button: pressing it shows the OUTDATED CODE error and
# no program is made, so old code can't be downloaded.
UPDATE_CHECK_STRICT = 1

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
# Stop the build at link time, NOT here while the makefile is being read: a $(error) at
# read time breaks every `make` call, including the ones the VEX extension runs to load
# the project, and that hides the Build button. This way the makefile always loads, the
# Build button stays, and pressing Build fails with this message and makes no .bin.
#
# It also opens a popup window so nobody misses it (Mac: system alert, Windows: message box,
# Linux: zenity/notify-send if installed). The popup text has no quotes or apostrophes on
# purpose, so it survives every shell. If a popup can't open, the build still stops with the error.
UC_POPUP_TITLE = Outdated code - build stopped
UC_POPUP_TEXT  = This code is $(UC_BEHIND) commit(s) behind GitHub ($(UC_UPSTREAM)). Pull first: Source Control > Pull in VS Code, or run git pull. Then build again. Nothing was built, so the old code was NOT downloaded to the robot.
ifeq ($(OS),Windows_NT)
UC_POPUP = powershell -NoProfile -Command "Add-Type -AssemblyName PresentationFramework; [void][System.Windows.MessageBox]::Show('$(UC_POPUP_TEXT)', '$(UC_POPUP_TITLE)', 'OK', 'Error')" >$(UC_NULL) 2>&1
else ifeq ($(shell uname -s),Darwin)
UC_POPUP = osascript -e 'display alert "$(UC_POPUP_TITLE)" message "$(UC_POPUP_TEXT)" as critical' >$(UC_NULL) 2>&1 &
else
UC_POPUP = (zenity --error --title='$(UC_POPUP_TITLE)' --text='$(UC_POPUP_TEXT)' || notify-send '$(UC_POPUP_TITLE)' '$(UC_POPUP_TEXT)') >$(UC_NULL) 2>&1 &
endif
.DEFAULT_GOAL := all
.PHONY: update-check-outdated
update-check-outdated:
	$(shell $(UC_POPUP))$(error $(UC_MSG))
$(BUILD)/$(PROJECT).elf: update-check-outdated
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
