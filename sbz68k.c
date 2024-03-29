//=====================================================================
// This is a C API for the 68k version of the decompressor.
// Feel free to modify it to fit your needs.
//=====================================================================
// Copyright (C) 2024 Ian Karlsson
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any
// damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any
// purpose, including commercial applications, and to alter it and
// redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must
//    not claim that you wrote the original software. If you use this
//    software in a product, an acknowledgment in the product
//    documentation would be appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must
//    not be misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source
//    distribution.
//=====================================================================

//! Decompress data using SBZ algorithm.
/*! \param   in   Pointer to input data. Must be word aligned!
 *  \param   out  Pointer to destination memory address.
 *  \return       Pointer to the end of decompressed data.
 */
unsigned char* SBZ_decompress(const void* in, unsigned char* out)
{
	static const unsigned short asm_blob[] = {
		0x7400,0x3458,0xd5c8,0x49fa,0x006a,0x703e,0x323c,0x8000,
		0x4ed4,0x3602,0xc400,0xd643,0xd643,0x161a,0x2649,0x96c3,
		0x4efb,0x2008,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,
		0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,
		0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,
		0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,
		0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x4ed4,
		0x12da,0xd241,0x6604,0x3218,0xd341,0x64f4,0xd241,0x6604,
		0x3218,0xd341,0x640c,0x141a,0xd402,0x6486,0x6a18,0x4efb,
		0x20f6,0x141a,0x76f8,0xc602,0x9403,0xd402,0xe643,0x2649,
		0xd6c3,0x4efb,0x20b8,0xd402,0x3618,0xb702,0xe44b,0x2649,
		0x96c3,0x7622,0xd403,0x6548,0x12db,0x12db,0x12db,0x12db,
		0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,
		0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,
		0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,
		0x12db,0x12db,0x12db,0x12db,0x12db,0x12db,0xd403,0x64b8,
		0xd402,0x4ef4,0x20b2,0x4e75,0x12da,0x12da,0x12da,0x12da,
		0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,
		0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,
		0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,
		0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,
		0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,
		0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,
		0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,
		0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,0x12da,
		0x12da,0x12da,0x12da,0x4ed4
	};

	register const void* a0 asm ("a0") = in;
	register unsigned char* a1 asm ("a1") = out;

	asm volatile (
		"jsr %2"
		: "+a" (a1)
		: "a" (a0), "m" (asm_blob)
		: "a2","a3","a4","d0","d1","d2","d3","cc"
	);

	return a1;
}
// vim: set sw=4 ts=4 noet:
