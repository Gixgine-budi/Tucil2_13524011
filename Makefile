# Makefile for Tucil2_13524011
# Compiles all Go programs (via go.work workspace) and places executables in bin/

BIN_DIR := bin
GO_PROGRAMS := ttov

.PHONY: all build run clean

all: build

build:
	@mkdir -p $(BIN_DIR)
	@for prog in $(GO_PROGRAMS); do \
		echo "Building $$prog..."; \
		go build -o $(BIN_DIR)/$$prog ./src/$$prog; \
	done

run: build
	./$(BIN_DIR)/ttov

clean:
	rm -rf $(BIN_DIR)
