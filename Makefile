# This makefile is for building the tool only.
# For the decompression library, see sbz68k.c
CFLAGS = -Os -s -DSBZ_TOOL

sbz: sbz.c

clean:
	rm sbz
.PHONY: clean
