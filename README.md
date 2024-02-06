# SBZ
SBZ is a fast LZSS-type compression algorithm designed for 68000
processors.

Typical decompression performance is about 260 kb/s on Mega Drive.

Compression ratio depends on the source data but should be roughly
comparable to the "Kosinski" algorithm.

## How to use
### Building the command line tool `sbz`
Easiest way is to simply run `make`. (Don't use the SGDK toolchain for
this, though. You need a native C toolchain, such as MinGW-w64.)

It is also possible to use `sbz.c` as a single-file library in your own
C programs, see the source code for details.

### Compressing files using `sbz`
To compress:

	sbz <input.bin> <output.sbz>

(It is also possible to drag and drop files to compress, resulting
 files will keep the original filename and extension but with `.sbz`
 appended).

To decompress:

	sbz -d <input.sbz> <output.bin>

### Data alignment
SBZ compressed files must be **WORD ALIGNED** in 68000 memory.

If you use rescomp, make sure that the align value is set to 2.

If you use an assembler, make sure to add an `even` directive before
`incbin`ing any SBZ files.

### Decompressing files using `sbz68k.c` (for 68000 C compilers)
Note: The C/ASM blob implementation has been tested with GCC 6.3.0 and
should also work with later versions.

Add "sbz68k.c" and "sbz68k.h" to your project. Include "sbz68k.h" and
use the function `SBZ_decompress` to decompress. The return value is
a pointer to the byte following the last decompressed byte.

### License
Zlib license.

