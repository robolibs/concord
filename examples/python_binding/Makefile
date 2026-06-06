SHELL := /bin/bash

ROOT := ../..
PYTHON ?= python3
VENV := .venv
VENV_PYTHON := $(CURDIR)/$(VENV)/bin/python

.PHONY: all develop basic main clean

all: basic

$(VENV_PYTHON):
	@$(PYTHON) -m venv $(VENV)

develop: $(VENV_PYTHON)
	@cd $(ROOT) && VIRTUAL_ENV=$(CURDIR)/$(VENV) PATH=$(CURDIR)/$(VENV)/bin:$$PATH PYO3_PYTHON=$(VENV_PYTHON) maturin develop --features python

basic: develop
	@$(VENV_PYTHON) basic.py

main: develop
	@$(VENV_PYTHON) main.py

clean:
	@rm -rf $(VENV)
