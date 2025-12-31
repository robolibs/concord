SHELL := /bin/bash

# ==================================================================================================
# Project configuration
# ==================================================================================================
PROJECT_NAME := $(shell cat NAME 2>/dev/null | tr -d '[:space:]')
ifeq ($(PROJECT_NAME),)
    $(error Error: NAME file not found or empty)
endif

PROJECT_CAP  := $(shell echo $(PROJECT_NAME) | tr '[:lower:]' '[:upper:]')
LATEST_TAG   ?= $(shell git describe --tags --abbrev=0 2>/dev/null)
TOP_DIR      := $(CURDIR)
BUILD_DIR    := $(TOP_DIR)/build

# ==================================================================================================
# Compiler selection: CC=gcc|clang (optional)
# ==================================================================================================
CC ?=
ifdef CC
    ifeq ($(CC),gcc)
        CMAKE_COMPILER_FLAG := -DCOMPILER=gcc
        XMAKE_COMPILER_FLAG := --toolchain=gcc
    else ifeq ($(CC),clang)
        CMAKE_COMPILER_FLAG := -DCOMPILER=clang
        XMAKE_COMPILER_FLAG := --toolchain=clang
    endif
endif

# ==================================================================================================
# Build system detection: BUILD_SYSTEM env > cmake > zig > xmake
# ==================================================================================================
ifndef BUILD_SYSTEM
    HAS_CMAKE := $(shell command -v cmake 2>/dev/null)
    HAS_CMAKE_FILE := $(shell [ -f CMakeLists.txt ] && echo "yes" || echo "")
    HAS_ZIG := $(shell command -v zig 2>/dev/null)
    HAS_ZIG_BUILD := $(shell [ -f build.zig ] && echo "yes" || echo "")
    HAS_XMAKE := $(shell command -v xmake 2>/dev/null)
    HAS_XMAKE_LUA := $(shell [ -f xmake.lua ] && echo "yes" || echo "")

    ifeq ($(and $(HAS_CMAKE),$(HAS_CMAKE_FILE)),yes)
        BUILD_SYSTEM := cmake
    else ifeq ($(and $(HAS_ZIG),$(HAS_ZIG_BUILD)),yes)
        BUILD_SYSTEM := zig
    else ifeq ($(and $(HAS_XMAKE),$(HAS_XMAKE_LUA)),yes)
        BUILD_SYSTEM := xmake
    else
        BUILD_SYSTEM := cmake
    endif
endif

# ==================================================================================================
# Build system specific commands (defined as variables)
# ==================================================================================================
ifeq ($(BUILD_SYSTEM),zig)
    # Zig build system
    CMD_BUILD       := zig build -Dexamples=true -Dtests=true 2>&1 | tee "$(TOP_DIR)/.complog"
    CMD_CONFIG      := zig build --help >/dev/null 2>&1
    CMD_RECONFIG    := rm -rf .zig-cache zig-out $(BUILD_DIR) && zig build --help >/dev/null 2>&1
    CMD_CLEAN       := rm -rf .zig-cache zig-out $(BUILD_DIR)
    CMD_TEST        := zig build test -Dtests=true
    CMD_TEST_SINGLE  = ./zig-out/bin/$(TEST)
    CMD_QUICKFIX    := grep "error:" "$(TOP_DIR)/.complog" > "$(TOP_DIR)/.quickfix" || true

else ifeq ($(BUILD_SYSTEM),xmake)
    # XMake build system
    CMD_BUILD       := xmake -j$(shell nproc) -y 2>&1 | tee "$(TOP_DIR)/.complog"
    CMD_CONFIG      := xmake f --examples=y --tests=y $(XMAKE_COMPILER_FLAG) -y 2>&1 | tee "$(TOP_DIR)/.complog" && xmake project -k compile_commands
    CMD_RECONFIG    := rm -rf .xmake $(BUILD_DIR) && xmake f --examples=y --tests=y $(XMAKE_COMPILER_FLAG) -c -y 2>&1 | tee "$(TOP_DIR)/.complog" && xmake project -k compile_commands
    CMD_CLEAN       := xmake clean -a
    CMD_TEST        := xmake test
    CMD_TEST_SINGLE  = ./build/linux/$$(uname -m)/release/$(TEST)
    CMD_QUICKFIX    := grep "error:" "$(TOP_DIR)/.complog" > "$(TOP_DIR)/.quickfix" || true

else
    # CMake build system (default)
    CMD_BUILD       := cd $(BUILD_DIR) && make -j$(shell nproc) 2>&1 | tee "$(TOP_DIR)/.complog"
    CMD_CONFIG      := mkdir -p $(BUILD_DIR) && cd $(BUILD_DIR) && if [ -f Makefile ]; then make clean; fi && cmake -Wno-dev $(CMAKE_COMPILER_FLAG) -D$(PROJECT_CAP)_BUILD_EXAMPLES=ON -D$(PROJECT_CAP)_ENABLE_TESTS=ON .. 2>&1 | tee "$(TOP_DIR)/.complog"
    CMD_RECONFIG    := rm -rf $(BUILD_DIR) && mkdir -p $(BUILD_DIR) && cd $(BUILD_DIR) && cmake -Wno-dev $(CMAKE_COMPILER_FLAG) -D$(PROJECT_CAP)_BUILD_EXAMPLES=ON -D$(PROJECT_CAP)_ENABLE_TESTS=ON .. 2>&1 | tee "$(TOP_DIR)/.complog"
    CMD_CLEAN       := rm -rf $(BUILD_DIR)
    CMD_TEST        := cd $(BUILD_DIR) && ctest --verbose --output-on-failure
    CMD_TEST_SINGLE  = $(BUILD_DIR)/$(TEST)
    CMD_QUICKFIX    := grep "^$(TOP_DIR)" "$(TOP_DIR)/.complog" | grep -E "error:" > "$(TOP_DIR)/.quickfix" || true
endif

# ==================================================================================================
# Info
# ==================================================================================================
$(info ------------------------------------------)
$(info Project: $(PROJECT_NAME))
$(info Build System: $(BUILD_SYSTEM))
$(info Compiler: $(CC))
$(info ------------------------------------------)

.PHONY: build b config c reconfig run r test t help h clean docs release

# ==================================================================================================
# Build targets
# ==================================================================================================
build:
	@echo "Running clang-format on source files..."
	@find ./src ./include -name "*.cpp" -o -name "*.hpp" -o -name "*.h" | xargs clang-format -i
ifeq ($(BUILD_SYSTEM),cmake)
	@if [ ! -d "$(BUILD_DIR)" ]; then \
		echo "Build directory doesn't exist, running config first..."; \
		$(MAKE) config; \
	fi
endif
	@$(CMD_BUILD)
	@$(CMD_QUICKFIX)

b: build

config:
	@$(CMD_CONFIG)

c: config

reconfig:
	@$(CMD_RECONFIG)

clean:
	@echo "Cleaning build directory..."
	@$(CMD_CLEAN)
	@echo "Build directory cleaned."

# ==================================================================================================
# Run and test
# ==================================================================================================
run:
	@./build/main

r: run

TEST ?=

test:
	@if [ -n "$(TEST)" ]; then \
		$(CMD_TEST_SINGLE); \
	else \
		$(CMD_TEST); \
	fi

t: test

# ==================================================================================================
# Help
# ==================================================================================================
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
	@echo "Build system: $(BUILD_SYSTEM) (override with BUILD_SYSTEM=cmake|xmake|zig)"
	@echo "Compiler:     CC=gcc|clang (for cmake/xmake only)"
	@echo

h: help

# ==================================================================================================
# Documentation
# ==================================================================================================
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

# ==================================================================================================
# Release
# ==================================================================================================
TYPE ?= patch
HAS_REL := $(shell command -v git-rel 2>/dev/null)

release:
	@if [ -z "$(HAS_REL)" ]; then \
		echo "git-rel is not installed. Please install it first."; \
		exit 1; \
	fi
	@if [ -z "$(TYPE)" ]; then \
		echo "Release type not specified. Use 'make release TYPE=[patch|minor|major|m.m.p]'"; \
		exit 1; \
	fi
	@git rel $(TYPE)
