CC := gcc
CFLAGS := -O2 -Wall -Wextra -std=gnu99 -D_GNU_SOURCE
LDFLAGS := -lasound -lm

SRC := src/anc.c
OUT := build/anc

.PHONY: all clean run measure

all: $(OUT)

$(OUT): $(SRC)
	mkdir -p build
	$(CC) $(CFLAGS) -o $(OUT) $(SRC) $(LDFLAGS)

run: $(OUT)
	cd runtime && ../$(OUT) run

measure: $(OUT)
	cd runtime && ../$(OUT) measure

clean:
	rm -rf build
