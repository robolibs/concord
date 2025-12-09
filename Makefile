SHELL := /bin/bash

PROJECT_NAME := $(shell grep -Po 'set\s*\(\s*project_name\s+\K[^)]+' CMakeLists.txt)
PROJECT_CAP  := $(shell echo $(PROJECT_NAME) | tr '[:lower:]' '[:upper:]')
LATEST_TAG   ?= $(shell git describe --tags --abbrev=0 2>/dev/null)
TOP_DIR      := $(CURDIR)
BUILD_DIR    := $(TOP_DIR)/build

ifeq ($(PROJECT_NAME),)
$(error Error: project_name not found in CMakeLists.txt)
endif

$(info ------------------------------------------)
$(info Project: $(PROJECT_NAME))
$(info ------------------------------------------)

.PHONY: build b config c reconfig run r test t help h clean docs release


build:
	@if [ ! -d "$(BUILD_DIR)" ]; then \
		echo "Build directory doesn't exist, running config first..."; \
		$(MAKE) config; \
	fi
	@cd $(BUILD_DIR) && set -o pipefail && make -j$(shell nproc) 2>&1 | tee >(grep "^$(TOP_DIR)" | grep -E "error:" > "$(TOP_DIR)/.quickfix")

b: build

config:
	@mkdir -p $(BUILD_DIR)
	@cd $(BUILD_DIR) && if [ -f Makefile ]; then make clean; fi
	@echo "cmake -Wno-dev -D$(PROJECT_CAP)_BUILD_EXAMPLES=ON -D$(PROJECT_CAP)_ENABLE_TESTS=ON .."
	@cd $(BUILD_DIR) && cmake -Wno-dev -D$(PROJECT_CAP)_BUILD_EXAMPLES=ON -D$(PROJECT_CAP)_ENABLE_TESTS=ON ..

reconfig:
	@rm -rf $(BUILD_DIR)
	@mkdir -p $(BUILD_DIR)
	@echo "cmake -Wno-dev -D$(PROJECT_CAP)_BUILD_EXAMPLES=ON -D$(PROJECT_CAP)_ENABLE_TESTS=ON .."
	@cd $(BUILD_DIR) && cmake -Wno-dev -D$(PROJECT_CAP)_BUILD_EXAMPLES=ON -D$(PROJECT_CAP)_ENABLE_TESTS=ON ..

c: config

run:
	@./build/main

r: run

test:
	@cd $(BUILD_DIR) && ctest --verbose --output-on-failure || true

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
	@echo "  test         Run tests"
	@echo "  docs         Build documentation (TYPE=mdbook|doxygen)"
	@echo "  release      Create a new release (TYPE=patch|minor|major)"
	@echo

h : help

clean:
	@echo "Cleaning build directory..."
	@rm -rf $(BUILD_DIR)
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


release:
	@if [ -z "$(TYPE)" ]; then \
		echo "Release type not specified. Use 'make release TYPE=[patch|minor|major]'"; \
		exit 1; \
	fi; \
	CURRENT_VERSION=$$(grep -E '^project\(.*VERSION [0-9]+\.[0-9]+\.[0-9]+' CMakeLists.txt | sed -E 's/.*VERSION ([0-9]+\.[0-9]+\.[0-9]+).*/\1/'); \
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
	sed -i -E 's/(project\(.*VERSION )[0-9]+\.[0-9]+\.[0-9]+/\1'$$version'/' CMakeLists.txt; \
	git add -A && git commit -m "chore(release): prepare for $$version"; \
	echo "$$changelog"; \
	git tag -a $$version -m "$$version" -m "$$changelog"; \
	git push --follow-tags --force --set-upstream origin develop; \
	gh release create $$version --notes "$$changelog"
