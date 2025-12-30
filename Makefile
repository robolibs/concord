SHELL := /bin/bash

# Detect build system: use BUILD_SYSTEM env if set, else prefer xmake if available, fallback to cmake
ifdef BUILD_SYSTEM
    # Use the environment-provided BUILD_SYSTEM
else
    HAS_XMAKE := $(shell command -v xmake 2>/dev/null)
    HAS_XMAKE_LUA := $(shell [ -f xmake.lua ] && echo "yes" || echo "")
    ifeq ($(and $(HAS_XMAKE),$(HAS_XMAKE_LUA)),yes)
        BUILD_SYSTEM := xmake
    else
        BUILD_SYSTEM := cmake
    endif
endif

PROJECT_NAME := $(shell cat NAME 2>/dev/null | tr -d '[:space:]')
ifeq ($(PROJECT_NAME),)
    $(error Error: NAME file not found or empty)
endif

PROJECT_CAP  := $(shell echo $(PROJECT_NAME) | tr '[:lower:]' '[:upper:]')
LATEST_TAG   ?= $(shell git describe --tags --abbrev=0 2>/dev/null)
TOP_DIR      := $(CURDIR)
BUILD_DIR    := $(TOP_DIR)/build

$(info ------------------------------------------)
$(info Project: $(PROJECT_NAME))
$(info Build System: $(BUILD_SYSTEM))
$(info ------------------------------------------)

.PHONY: build b config c reconfig run r test t help h clean docs release


build:
	@echo "Running clang-format on source files..."
	@find ./src ./include -name "*.cpp" -o -name "*.hpp" -o -name "*.h" | xargs clang-format -i
ifeq ($(BUILD_SYSTEM),xmake)
	@xmake -j$(shell nproc) -y 2>&1 | tee "$(TOP_DIR)/.complog"
	@grep "error:" "$(TOP_DIR)/.complog" > "$(TOP_DIR)/.quickfix" || true
else
	@if [ ! -d "$(BUILD_DIR)" ]; then \
		echo "Build directory doesn't exist, running config first..."; \
		$(MAKE) config; \
	fi
	@cd $(BUILD_DIR) && set -o pipefail && make -j$(shell nproc) 2>&1 | tee "$(TOP_DIR)/.complog"
	@grep "^$(TOP_DIR)" "$(TOP_DIR)/.complog" | grep -E "error:" > "$(TOP_DIR)/.quickfix" || true
endif

b: build

config:
ifeq ($(BUILD_SYSTEM),xmake)
	@xmake f --examples=y --tests=y -y 2>&1 | tee "$(TOP_DIR)/.complog"
	@xmake project -k compile_commands
else
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && if [ -f Makefile ]; then make clean; fi
	@echo "cmake -Wno-dev -D$(PROJECT_CAP)_BUILD_EXAMPLES=ON -D$(PROJECT_CAP)_ENABLE_TESTS=ON .."
	@cd $(BUILD_DIR) && cmake -Wno-dev -D$(PROJECT_CAP)_BUILD_EXAMPLES=ON -D$(PROJECT_CAP)_ENABLE_TESTS=ON .. 2>&1 | tee "$(TOP_DIR)/.complog"
endif

reconfig:
ifeq ($(BUILD_SYSTEM),xmake)
	@rm -rf .xmake $(BUILD_DIR)
	@xmake f --examples=y --tests=y -c -y 2>&1 | tee "$(TOP_DIR)/.complog"
	@xmake project -k compile_commands
else
	@rm -rf $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)
	@echo "cmake -Wno-dev -D$(PROJECT_CAP)_BUILD_EXAMPLES=ON -D$(PROJECT_CAP)_ENABLE_TESTS=ON .."
	@cd $(BUILD_DIR) && cmake -Wno-dev -D$(PROJECT_CAP)_BUILD_EXAMPLES=ON -D$(PROJECT_CAP)_ENABLE_TESTS=ON .. 2>&1 | tee "$(TOP_DIR)/.complog"
endif

c: config

run:
	@./build/main

r: run

TEST ?=

test:
ifeq ($(BUILD_SYSTEM),xmake)
	@if [ -n "$(TEST)" ]; then \
		./build/linux/$$(uname -m)/release/$(TEST); \
	else \
		xmake test; \
	fi
else
	@if [ -n "$(TEST)" ]; then \
		$(BUILD_DIR)/$(TEST); \
	else \
		cd $(BUILD_DIR) && ctest --verbose --output-on-failure; \
	fi
endif

t: test

help:
	@echo
	@echo "Usage: make [target]"
	@echo
	@echo "Available targets:"
	@echo "  build        Build project"
	@echo "  config       Configure and generate build files (preserves cache)"
	@echo "  reconfig     Full reconfigure (cleans everything including cache)"
	@echo "  run          Run the main executable"
	@echo "  test         Run tests (TEST=<name> to run specific test)"
	@echo "  docs         Build documentation (TYPE=mdbook|doxygen)"
	@echo "  release      Create a new release (TYPE=patch|minor|major)"
	@echo

h : help

clean:
	@echo "Cleaning build directory..."
ifeq ($(BUILD_SYSTEM),xmake)
	@xmake clean -a
else
	@rm -rf $(BUILD_DIR)
endif
	@echo "Build directory cleaned."

docs:
ifeq ($(TYPE),mdbook)
	@command -v mdbook >/dev/null 2>&1 || { echo "mdbook is not installed. Please install it first."; exit 1; }
	@mdbook build $(TOP_DIR)/book --dest-dir $(TOP_DIR)/docs
	@git add --all && git commit -m "docs: building website/mdbook"
else ifeq ($(TYPE),doxygen)
	@command -v doxygen >/dev/null 2>&1 || { echo "doxygen is not installed. Please install it first."; exit 1; }
else
	$(error Invalid documentation type. Use 'make docs TYPE=mdbook' or 'make docs TYPE=doxygen')
endif

TYPE ?= patch
HAS_CODENAME := $(shell command -v git-codename 2>/dev/null)

release:
	@if [ -z "$(TYPE)" ]; then \
		echo "Release type not specified. Use 'make release TYPE=[patch|minor|major]'"; \
		exit 1; \
	fi; \
	CURRENT_VERSION=$$(cat VERSION | tr -d '[:space:]'); \
	IFS='.' read -r MAJOR MINOR PATCH <<< "$$CURRENT_VERSION"; \
	case "$(TYPE)" in \
		major) MAJOR=$$((MAJOR+1)); MINOR=0; PATCH=0 ;; \
		minor) MINOR=$$((MINOR+1)); PATCH=0 ;; \
		patch) PATCH=$$((PATCH+1)); ;; \
		*) echo "Invalid release type. Use patch, minor or major."; exit 1 ;; \
	esac; \
	version="$$MAJOR.$$MINOR.$$PATCH"; \
	if [ -n "$(LATEST_TAG)" ]; then \
		changelog=$$(git cliff $(LATEST_TAG)..HEAD --strip all); \
		git cliff --tag $$version $(LATEST_TAG)..HEAD --prepend CHANGELOG.md; \
	else \
		changelog=$$(git cliff --unreleased --strip all); \
		git cliff --tag $$version --unreleased --prepend CHANGELOG.md; \
	fi; \
	if { [ "$(TYPE)" = "minor" ] || [ "$(TYPE)" = "major" ]; } && [ -n "$(HAS_CODENAME)" ]; then \
		RELEASE_NAME=$$(git-codename "$$changelog"); \
		echo "-------------- $$RELEASE_NAME --------------"; \
	fi; \
	echo "$$version" > VERSION; \
	git add -A && git commit -m "chore(release): prepare for $$version"; \
	echo "$$changelog"; \
	git tag -a $$version -m "$$version" -m "$$changelog"; \
	git push --follow-tags --force --set-upstream origin develop; \
	gh release create $$version --notes "$$changelog"
