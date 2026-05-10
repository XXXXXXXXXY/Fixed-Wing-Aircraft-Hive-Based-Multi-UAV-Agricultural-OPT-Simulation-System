CC ?= gcc
CFLAGS ?= -std=c11 -Wall -Wextra -pedantic -O2

BIN := scout_opt
SRC := c_src/main.c c_src/scout_opt.c c_src/config_loader.c c_src/sitl_bridge.c c_src/diagnostics.c c_src/visual_export.c
INC := -Ic_include

.PHONY: all run clean

all: $(BIN)

$(BIN): $(SRC) c_include/scout_opt.h c_include/scout_opt_config.h c_include/scout_opt_sitl.h
	$(CC) $(CFLAGS) $(INC) -o $(BIN) $(SRC) -lm

run: $(BIN)
	./$(BIN)

clean:
	rm -f $(BIN) $(BIN).exe
