# SPDX-License-Identifier: GPL-3.0-only
# Host (Mac/Linux) checks of the Passport build -- no ESP-IDF needed.
#   make test    decoder bench: the Passport decoder vs the iPhone/Watch float
#                decoder on the same encoded audio (needs APPLE_CORE), and the
#                firmware app run on a simulated screen and microphone
#   make bench   the Passport decoder alone
CC        ?= cc
CFLAGS    ?= -std=gnu99 -O2 -g -Wall -Wextra -Wno-unused-parameter -D_DEFAULT_SOURCE
CORE       = ../main/sstv_dsp.c
BSP_INC   ?= ../third_party/ai-passport/components/bsp/include
APPLE_CORE ?= ../../apricot-sstv-apple/Shared/Core

all: bench_port sim_app

bench_port: bench_port.c bench_common.h enc.c util.c $(CORE) ../main/sstv_dsp.h
	$(CC) $(CFLAGS) -I../main -DSSTV_POOL -o $@ bench_port.c enc.c util.c $(CORE) -lm

bench_orig: bench_orig.c bench_common.h enc.c util.c
	$(CC) $(CFLAGS) -I$(APPLE_CORE) -o $@ bench_orig.c enc.c util.c $(APPLE_CORE)/sstv_dsp.c -lm

# the firmware's app_main.c, against stand-ins for the ESP-IDF headers and a
# simulated panel / mic / keys (sim_app.c); ASan + UBSan on
# the splash goes in as ESP-IDF's EMBED_FILES would (GNU ld; Linux)
splash.o: ../main/splash.rgb565 ../main/help_en.rgb565 ../main/help_zh.rgb565
	cd ../main && ld -r -b binary -z noexecstack -o ../host/splash.o splash.rgb565 help_en.rgb565 help_zh.rgb565

sim_app: sim_app.c ../main/app_main.c $(CORE) enc.c util.c splash.o
	$(CC) $(CFLAGS) -fsanitize=address,undefined -DRD_BUDGET=4096 -Iidf_stub -I$(BSP_INC) -I../main -I. \
	    -o $@ sim_app.c ../main/app_main.c $(CORE) enc.c util.c splash.o -lm -Wl,-z,noexecstack

out:
	mkdir -p out

bench: bench_port out
	./bench_port out

test: bench_port sim_app out
	./bench_port out
	@if [ -f $(APPLE_CORE)/sstv_dsp.c ]; then $(MAKE) bench_orig && echo "--- float baseline:" && ./bench_orig; fi
	./sim_app out 44 8
	@echo "screens: out/screen_*.ppm"

clean:
	rm -f bench_port bench_orig sim_app splash.o out/*.ppm out/*.png
.PHONY: all bench test clean
