SHELL := /bin/bash

PROJECT_NAME := $(shell sed -n '/^[[:space:]]*[^#\[[:space:]]/p' PROJECT | head -1 | tr -d '[:space:]')
PROJECT_VERSION := $(shell sed -n '/^[[:space:]]*[^#\[[:space:]]/p' PROJECT | sed -n '2p' | tr -d '[:space:]')
ifeq ($(PROJECT_NAME),)
    $(error Error: PROJECT file not found or invalid)
endif

TOP_DIR := $(CURDIR)
CARGO := cargo
EXAMPLE ?= main

$(info ------------------------------------------)
$(info Project: $(PROJECT_NAME) v$(PROJECT_VERSION))
$(info ------------------------------------------)

.PHONY: build b compile c run r test t check fmt bench clean help h

build:
	@$(CARGO) build --lib --examples

b: build

compile:
	@$(CARGO) clean
	@$(MAKE) build

c: compile

run:
	@$(CARGO) run --example $(EXAMPLE)

r: run

test:
	@$(CARGO) test --all-targets

t: test

check:
	@$(CARGO) check --all-targets

fmt:
	@$(CARGO) fmt --all

bench:
	@$(CARGO) bench --bench core_workloads -- --quick

clean:
	@$(CARGO) clean

help:
	@echo
	@echo "Usage: make [target]"
	@echo
	@echo "Available targets:"
	@echo "  build        Build the library and examples"
	@echo "  compile      Clean and rebuild"
	@echo "  run          Run a development example (EXAMPLE=main by default)"
	@echo "  test         Run all tests"
	@echo "  check        Run cargo check on all targets"
	@echo "  fmt          Format the workspace"
	@echo "  bench        Run the custom benchmark target"
	@echo "  clean        Remove Cargo build artifacts"
	@echo
	@echo "Examples:"
	@echo "  make run"
	@echo "  make run EXAMPLE=simple_graph"
	@echo

h: help
