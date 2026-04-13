SHELL := /bin/bash

ROOT := ../..
C_DEMO_BIN := $(ROOT)/target/debug/concord_c_api_demo

.PHONY: all build run clean

all: run

build:
	@cd $(ROOT) && cargo build
	@cc -I$(ROOT)/include demo.c -L$(ROOT)/target/debug -lconcord -Wl,-rpath,$$(cd $(ROOT) && pwd)/target/debug -lm -o $(C_DEMO_BIN)

run: build
	@$(C_DEMO_BIN)

clean:
	@rm -f $(C_DEMO_BIN)
