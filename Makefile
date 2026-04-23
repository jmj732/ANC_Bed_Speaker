CC := gcc
OPTFLAGS ?= -O3 -mcpu=cortex-a76
CFLAGS := $(OPTFLAGS) -Wall -Wextra -std=gnu99 -D_GNU_SOURCE -DDEVICE=\"hw:sndrpihifiberry,0\"
LDFLAGS := -lasound -lm

LIB_SRC := src/anc_run.c src/anc_nb.c src/anc_pass.c src/anc_sim.c

OUT := build/anc
SNORE_OUT := build/snore_anc

.PHONY: all clean run measure run-snore measure-snore

all: $(OUT) $(SNORE_OUT)

$(OUT): src/anc_main.c src/anc_app.c $(LIB_SRC)
	mkdir -p build
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

$(SNORE_OUT): src/snore_anc.c src/anc_app.c $(LIB_SRC)
	mkdir -p build
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

run: $(OUT)
	cd runtime && ../$(OUT) run

measure: $(OUT)
	cd runtime && ../$(OUT) measure

run-snore: $(SNORE_OUT)
	cd runtime && ../$(SNORE_OUT) run

measure-snore: $(SNORE_OUT)
	cd runtime && ../$(SNORE_OUT) measure

clean:
	rm -rf build
