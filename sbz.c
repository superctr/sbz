//=====================================================================
// SBZ compression / decompression library and utility.
// Note this version of the library is not designed to be embedded
// in 68k applications, use sbz68k.c for this.
//---------------------------------------------------------------------
// To include as a library:
//   #define SBZ_HEADER_ONLY
//   #include "sbz.c"
//---------------------------------------------------------------------
// To compile as a library
//   gcc -O2 -o sbz.o -c sbz.c
//---------------------------------------------------------------------
// To compile as an executable
//   gcc -O2 -DSBZ_TOOL -o sbz sbz.c
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

// ====================================================================
// Beginning of SBZ library header
// ====================================================================
#ifndef SBZ_HEADER_INCLUDED
#define SBZ_HEADER_INCLUDED

#include <stdio.h>
#include <stdint.h>

// Maximum size of compressed data
#define MAX_SBZ_SIZE  0x1000000

enum sbz_status
{
	// Input data underflow. Not a critical error
	SBZ_UNDERFLOW = 1,
	// Compression/decompression ok
	SBZ_OK = 0,
	// Memory allocation error
	SBZ_ERROR = -1,
	// Missing EOF token
	SBZ_INCOMPLETE = -2,
	// Not enough data in input buffer
	SBZ_OVERFLOW = -3,
	// Copy offset invalid
	SBZ_INVALID_OFFSET = -4,
	// Copy length invalid
	SBZ_INVALID_LENGTH = -5,
	// Verify error
	SBZ_VERIFY_ERROR = -6
};

enum sbz_status sbz_compress(const uint8_t *in_buf, uint32_t in_size, uint8_t **out_buf, uint32_t *out_size);
enum sbz_status sbz_decompress(const uint8_t *in_buf, uint32_t in_size, uint8_t **out_buf, uint32_t *out_size);

enum sbz_status sbz_verify(const uint8_t *in_buf, uint32_t in_size, const uint8_t *blz_buf, uint32_t blz_size);

enum sbz_status sbz_dump_nodes(const uint8_t *in_buf, uint32_t in_size, FILE* file);
const char* sbz_get_decompress_message(enum sbz_status status);

#endif

// ====================================================================
// Beginning of SBZ library source code
// ====================================================================
#ifndef SBZ_HEADER_ONLY
#ifndef SBZ_LIB_INCLUDED
#define SBZ_LIB_INCLUDED

#include <stdlib.h>
#include <string.h>
#include <assert.h>

// Absolute max length of copy command.
static const uint32_t min_copy_l_length = 3;
static const uint32_t max_copy_l_length = 34;   // Max size of copy long command
static const uint32_t max_copy_l_offset = 0x3ff;

static const uint32_t min_copy_e_length = 4;
static const uint32_t max_copy_e_length = 259;   // Max size of copy extended command
static const uint32_t max_copy_e_offset = 0x3fff;

static const uint32_t min_copy_s_length = 2;
static const uint32_t max_copy_s_length = 9;    // Max size of copy short command
static const uint32_t max_copy_s_offset = 0x20;

// for a future (uncompatible) change, this could be increased to 72. I have tried, but
// it didn't really reduce the file size in my test files. 73 is also possible, but will
// probably just result in higher decompression CPU load.
static const uint32_t max_direct_length = 71; // Max size of direct command

#define MIN(a,b) ((a) < (b) ? (a) : (b))
#define MAX(a,b) ((a) > (b) ? (a) : (b))

enum sbz_strategy {
	S_INVALID = 0,
	S_EXTENDED = 1,
	S_LONG = 2,
	S_SHORT = 3,
	S_DIRECT = 4,
	S_IMMEDIATE = 5
};

struct sbz_node
{
	enum sbz_strategy strategy : 8;
	uint32_t origin; // location of previous node
	uint32_t weight; // Number of bits used
	uint32_t desc_bits; // Number of description bits used
	uint16_t target;  // Next node in the optimal path
	uint16_t offset; // window search position (for the copy strategy)
};

struct sbz
{
	int use_extended_copy;
	uint32_t max_copy_offset;
	uint32_t max_copy_length;

	// Node table. For each node, we save the best path that brings us to it.
	struct sbz_node* nodes;
	const uint8_t *input_buf;
	uint32_t input_size;
	int16_t kmp_table[260];

	// pointer to compression output buffer
	uint8_t *output_buf;
	uint8_t *temp_buf;
	uint32_t output_size;
};

// Get the weight (size in bits) of the given strategy and copy length
static inline uint32_t get_weight(enum sbz_strategy strategy, uint32_t length)
{
	uint32_t result = 9999;
	switch(strategy)
	{
		case S_IMMEDIATE:
			result = length * 9;
			break;
		case S_DIRECT:
			result = 10 + length * 8;
			break;
		case S_SHORT:
			result = 10;
			break;
		case S_LONG:
			result = 18;
			break;
		case S_EXTENDED:
			result = 26;
			break;
		case S_INVALID:
			break;
	}
	return result;
}

// Get the number of description bits needed by the given strategy
static inline uint32_t get_desc_bits(enum sbz_strategy strategy, uint32_t length)
{
	uint32_t result = 9999;
	switch(strategy)
	{
		case S_IMMEDIATE:
			result = length;
			break;
		case S_DIRECT:
			result = 2;
			break;
		case S_SHORT:
			result = 2;
			break;
		case S_LONG:
			result = 2;
			break;
		case S_EXTENDED:
			result = 18;
			break;
		case S_INVALID:
			break;
	}
	return result;
}

// Check if a given path is optimal to the target node
static inline int check_path(struct sbz *sbz, struct sbz_node *target, enum sbz_strategy strategy, uint32_t weight)
{
	int result = 0;

	// Check that the target node is actually valid
	assert(target <= &sbz->nodes[sbz->input_size]);

	// Never add an invalid path
	if(strategy == S_INVALID)
		result = 0;

	// Always overwrite an invalid path
	if(target->strategy == S_INVALID)
		result = 1;

	// Fewer bytes
	if(weight < target->weight)
		result = 1;

	// Greater strategy (i.e. less CPU cycles required to decompress)
	else if(weight == target->weight && strategy > target->strategy)
		result = 1;

	return result;
}

// Decide the optimal direct copy strategy given the current offset and length
static inline enum sbz_strategy decide_direct_strategy(uint32_t length)
{
	enum sbz_strategy result = S_INVALID;
	if(length <= 8)
		result = S_IMMEDIATE;
	else if(length <= max_direct_length)
		result = S_DIRECT;
	return result;
}

// Add all direct paths possible from the origin.
static inline void add_direct_paths(struct sbz *sbz, uint32_t origin)
{
	uint32_t max_length = MIN(max_direct_length, sbz->input_size - origin);
	uint32_t origin_weight = sbz->nodes[origin].weight;
	uint32_t origin_desc_bits = sbz->nodes[origin].desc_bits;

	for(uint32_t length = 1; length <= max_length; length++)
	{
		struct sbz_node *target = &sbz->nodes[origin + length];
		enum sbz_strategy strategy = decide_direct_strategy(length);
		uint32_t weight = origin_weight + get_weight(strategy, length);

		if(check_path(sbz, target, strategy, weight))
		{
			target->strategy = strategy;
			target->origin = origin;
			target->weight = weight;
			target->desc_bits = origin_desc_bits + get_desc_bits(strategy, length);
		}
	}
}

// Build the KMP search table. Algorithm taken from wikipedia
static inline void make_kmp_table(struct sbz *sbz, const uint8_t *string, uint32_t length)
{
	uint32_t pos = 1;
	int16_t *kmp_table = sbz->kmp_table, cnd = 0;
	kmp_table[0] = -1;
	for(; pos < length; pos++, cnd++)
	{
		if(string[pos] == string[cnd])
		{
			kmp_table[pos] = kmp_table[cnd];
		}
		else
		{
			kmp_table[pos] = cnd;
			do
			{
				cnd = kmp_table[cnd];
			}
			while(cnd >= 0 && string[pos] != string[cnd]);
		}
	}
	kmp_table[pos] = cnd;
}

// Decide the optimal window copy strategy given the current offset and length
static inline enum sbz_strategy decide_copy_strategy(struct sbz *sbz, uint32_t offset, uint32_t length)
{
	enum sbz_strategy result = S_INVALID;
	if(   offset <= max_copy_s_offset
	   && length <= max_copy_s_length
	   && length >= min_copy_s_length)
		result = S_SHORT;
	else if(   offset <= max_copy_l_offset
			&& length <= max_copy_l_length
			&& length >= min_copy_l_length)
		result = S_LONG;
	else if(   sbz->use_extended_copy
			&& offset <= max_copy_e_offset
			&& length <= max_copy_e_length
			&& length >= min_copy_e_length)
		result = S_EXTENDED;
	return result;
}

// Checks if a given path has the shortest length to the target and adds it if so
static inline void add_copy_path(struct sbz *sbz, uint32_t origin, uint32_t offset, uint32_t length)
{
	enum sbz_strategy strategy = decide_copy_strategy(sbz, offset, length);

	if(strategy != S_INVALID)
	{
		uint32_t weight = sbz->nodes[origin].weight + get_weight(strategy, length);
		struct sbz_node *target = &sbz->nodes[origin + length];

		if(check_path(sbz, target, strategy, weight))
		{
			target->strategy = strategy;
			target->origin = origin;
			target->offset = offset;
			target->weight = weight;
			target->desc_bits = sbz->nodes[origin].desc_bits + get_desc_bits(strategy, length);
		}
	}
}

// Add all copy paths possible from the origin
static inline void add_copy_paths(struct sbz *sbz, uint32_t origin)
{
	int32_t window_start = MAX(0, (int32_t)(origin - sbz->max_copy_offset));
	assert(window_start >= 0);
	uint32_t window_size = origin - window_start;
	uint32_t string_size = MIN(sbz->max_copy_length, sbz->input_size - origin);
	uint32_t overlap_size = sbz->input_size - window_start;
	const uint8_t *window = sbz->input_buf + window_start;
	const uint8_t *string = sbz->input_buf + origin;
	uint32_t window_pos = 0;
	uint16_t string_pos = 0;

	make_kmp_table(sbz, string, string_size);

	// Perform a KMP search through the sliding window
	while(window_pos < window_size)
	{
		if(window[window_pos] == string[string_pos])
		{
			string_pos++;
			window_pos++;
			add_copy_path(sbz, origin, window_size - window_pos + string_pos, string_pos);

			if(string_pos == string_size)
			{
				string_pos = sbz->kmp_table[string_pos];
			}
		}
		else
		{
			string_pos = sbz->kmp_table[string_pos];
			if((int16_t)string_pos < 0)
			{
				string_pos++;
				window_pos++;
			}
		}
	}

	// LZSS supports overlapping copy offset and destination, see if we have data left in the search string
	while(   window_size
		  && string_pos
		  && window_pos < overlap_size
		  && string_pos < string_size
		  && window[window_pos] == string[string_pos])
	{
		string_pos++;
		window_pos++;
		add_copy_path(sbz, origin, window_size - window_pos + string_pos, string_pos);
	}
}

// Add possible paths for each byte in the uncompressed file.
static void add_nodes(struct sbz *sbz)
{
	// The weight of the first node is the length of the EOF command.
	sbz->nodes[0].weight = 10+16;
	sbz->nodes[0].desc_bits = 18;
	for(uint32_t origin = 0; origin < sbz->input_size; origin++)
	{
		add_direct_paths(sbz, origin);
		add_copy_paths(sbz, origin);
	}
}

// Dump the graph (for debugging)
static inline void dump_nodes(struct sbz *sbz, FILE* file)
{
	for(uint32_t origin = 0; origin < sbz->input_size; origin++)
	{
		if(!sbz->nodes[origin].target)
			continue;
		fprintf(file, "to %5d [%02x]: from %5d [%02x], strategy %d offset %d length %d weight %d\n",
				origin,
				sbz->input_buf[origin],
				sbz->nodes[origin].origin,
				sbz->input_buf[sbz->nodes[origin].origin],
				sbz->nodes[origin].strategy,
				sbz->nodes[origin].offset,
				origin - sbz->nodes[origin].origin,
				sbz->nodes[origin].weight);
	}
	fprintf(file, "to eof:        from %5d [%02x], strategy %d offset %d length %d weight %d\n",
			sbz->nodes[sbz->input_size].origin,
			sbz->input_buf[sbz->nodes[sbz->input_size].origin],
			sbz->nodes[sbz->input_size].strategy,
			sbz->nodes[sbz->input_size].offset,
			sbz->input_size - sbz->nodes[sbz->input_size].origin,
			sbz->nodes[sbz->input_size].weight);
}

// Check that the parameters are valid for the chosen strategy
static inline void check_strategy(enum sbz_strategy strategy, uint32_t offset, uint32_t length)
{
	switch(strategy)
	{
		case S_EXTENDED:
			assert(length >= min_copy_e_length);
			assert(length <= max_copy_e_length);
			assert(offset > 0);
			assert(offset <= 0x3fff);
			break;
		case S_LONG:
			assert(length >= min_copy_l_length);
			assert(length <= max_copy_l_length);
			assert(offset > 0);
			assert(offset <= 0x3ff);
			break;
		case S_SHORT:
			assert(length >= min_copy_s_length);
			assert(length <= max_copy_s_length);
			assert(offset > 0);
			assert(offset <= 0x20);
			break;
		case S_IMMEDIATE:
			// Technically the immediate data has no length restriction, but
			// we put it here anyway since it doesn't make sense given that
			// the direct strategy is more efficient at that point.
			assert(length > 0);
			assert(length <= 8);
			break;
		case S_DIRECT:
			assert(length > 8);
			assert(length <= max_direct_length);
			break;
		case S_INVALID:
			break;
	}
}

static inline void push_desc_bit(uint8_t bit, uint8_t **head, uint8_t **tail, uint8_t *bits_left)
{
	if(!(*head))
	{
		*head = *tail;
		*tail = *tail + 2;
		(*head)[0] = bit << 7;
		(*head)[1] = 0;
		*bits_left = 15;
	}
	else
	{
		uint8_t next_bit = --(*bits_left);
		if (next_bit > 7)
			(*head)[0] |= (bit << (next_bit - 8));
		else
			(*head)[1] |= (bit << next_bit);
		if(!(*bits_left))
		{
			*head = NULL;
		}
	}
}

static inline void push_desc_word(uint16_t data, uint8_t **tail)
{
	uint8_t* buffer = *tail;
	*buffer++ = (data >> 8) & 0xff;
	*buffer++ = data & 0xff;
	*tail = buffer;
}

// Construct the nodes into a compressed file.
static inline void generate_output(struct sbz *sbz, uint32_t origin)
{
	struct sbz_node *node = &sbz->nodes[origin];
	struct sbz_node *last_node = &sbz->nodes[sbz->input_size];

	uint8_t *tail = sbz->temp_buf;
	uint8_t *desc_tail = sbz->output_buf + 2;

	uint8_t *head = NULL; // Pointer to the last description field
	uint8_t bits_left = 0;  // Number of bits left to write

	while(node <= last_node)
	{
		enum sbz_strategy strategy = node->strategy;
		uint32_t offset = node->offset;
		uint32_t length = origin - node->origin;

		check_strategy(strategy, offset, length);

		assert(tail < (sbz->temp_buf + sbz->output_size));

		switch(strategy)
		{
			case S_IMMEDIATE:
				offset = node->origin - offset;
				while(length--)
				{
					push_desc_bit(0, &head, &desc_tail, &bits_left);
					*tail++ = sbz->input_buf[offset++];
				}
				break;
			case S_DIRECT:
				offset = node->origin - offset;
				push_desc_bit(1, &head, &desc_tail, &bits_left);
				push_desc_bit(1, &head, &desc_tail, &bits_left);
				*tail++ = 0xc1 + (max_direct_length - length);
				while(length--)
					*tail++ = sbz->input_buf[offset++];
				break;
			case S_SHORT:
				push_desc_bit(1, &head, &desc_tail, &bits_left);
				push_desc_bit(0, &head, &desc_tail, &bits_left);
				*tail++ = ((32 - offset) << 3) + (max_copy_s_length - length);
				break;
			case S_LONG:
				push_desc_bit(1, &head, &desc_tail, &bits_left);
				push_desc_bit(1, &head, &desc_tail, &bits_left);
				*tail++ = 0x00 + ((offset & 0x300) >> 3) + (max_copy_l_length - length);
				*tail++ = offset & 0xff;
				break;
			case S_EXTENDED:
				push_desc_bit(1, &head, &desc_tail, &bits_left);
				push_desc_bit(1, &head, &desc_tail, &bits_left);
				*tail++ = 0x80 + (((max_copy_e_length - length) >> 2) ^ (offset & 0x3f));
				push_desc_word((offset << 2) | ((max_copy_e_length - length) & 3), &desc_tail);
				break;
			default:
				assert(0);
				break;
		}

		// Write the commands
		if(node == last_node)
		{
			// Add the EOF command
			push_desc_bit(1, &head, &desc_tail, &bits_left);
			push_desc_bit(1, &head, &desc_tail, &bits_left);
			*tail++ = 0xc0;

			// Calculate the length of the description bitstream
			int real_desc_length = desc_tail - sbz->output_buf;
			int actual_desc_length = real_desc_length;

			// The remaining bits are all unused, so we can overwrite the last byte.
			if (head == (desc_tail - 2) && bits_left > 7)
				actual_desc_length -= 1;

			// First word contains the # of bytes to skip to reach the command stream
			uint16_t desc_skip = actual_desc_length - 2;
			sbz->output_buf[0] = desc_skip >> 8;
			sbz->output_buf[1] = desc_skip & 0xff;

			// Append the command byte stream to the description byte stream
			memcpy(sbz->output_buf + actual_desc_length, sbz->temp_buf, tail - sbz->temp_buf);
			assert((real_desc_length + tail - sbz->temp_buf) == sbz->output_size);
			sbz->output_size = actual_desc_length + tail - sbz->temp_buf;
			break;
		}

		origin += node->target;
		node = &sbz->nodes[origin];
	}
}

// Trace backwards until we find the first node.
static inline uint32_t find_origin_node(struct sbz *sbz)
{
	uint32_t origin = sbz->input_size;
	struct sbz_node *node = &sbz->nodes[origin];

	while(node->origin != 0)
	{
		uint32_t target = origin - node->origin;

		assert(node > &sbz->nodes[0]);
		assert(node->origin < origin);

		origin = node->origin;

		node = &sbz->nodes[origin];
		node->target = target;
	}
	return origin;
}

// Main compression routine
static inline int compress(struct sbz *sbz)
{
	int result = -1;
	uint32_t byte_weight, word_weight;
	struct sbz_node *last_node;

	sbz->nodes = (struct sbz_node*)calloc(sbz->input_size + 1, sizeof(struct sbz_node));
	if(sbz->nodes)
	{
		add_nodes(sbz);

		last_node = &sbz->nodes[sbz->input_size];
		byte_weight = last_node->weight - last_node->desc_bits;
		word_weight = last_node->desc_bits;

		sbz->output_size =  (byte_weight >> 3) + ((byte_weight & 7) ? 1 : 0);
		sbz->output_size += ((word_weight >> 4) + ((word_weight & 15) ? 1 : 0)) << 1;

		sbz->output_buf = (uint8_t*)malloc(sbz->output_size);
		sbz->temp_buf = (uint8_t*)malloc(sbz->output_size);
		if(sbz->output_buf && sbz->temp_buf)
		{
			generate_output(sbz, find_origin_node(sbz));
			free(sbz->temp_buf);
			result = 0;
		}
		free(sbz->nodes);
	}
	return result;
}

enum sbz_status sbz_compress(const uint8_t *in_buf, uint32_t in_size, uint8_t **out_buf, uint32_t *out_size)
{
	enum sbz_status result = SBZ_ERROR;
	struct sbz sbz = {};

	sbz.use_extended_copy = 1;
	sbz.max_copy_length = max_copy_e_length;
	sbz.max_copy_offset = max_copy_e_offset;

	sbz.input_buf = in_buf;
	sbz.input_size = in_size;

	if(!compress(&sbz))
	{
		*out_buf = sbz.output_buf;
		*out_size = sbz.output_size;
		result = SBZ_OK;
	}

	return result;
}

enum sbz_status sbz_dump_nodes(const uint8_t *in_buf, uint32_t in_size, FILE* file)
{
	enum sbz_status result = SBZ_ERROR;
	struct sbz sbz = {};

	sbz.use_extended_copy = 1;
	sbz.max_copy_length = max_copy_e_length;
	sbz.max_copy_offset = max_copy_e_offset;

	sbz.input_buf = in_buf;
	sbz.input_size = in_size;

	sbz.nodes = (struct sbz_node*)calloc(sbz.input_size + 1, sizeof(struct sbz_node));
	if(sbz.nodes)
	{
		add_nodes(&sbz);
		find_origin_node(&sbz);
		dump_nodes(&sbz, file);
		free(sbz.nodes);
		result = SBZ_OK;
	}

	return result;
}
// Conditional memcpy. Supports overlapping regions if src > dst
static inline uint32_t my_memcpy(uint8_t *dst, const uint8_t *src, uint32_t size)
{
	uint32_t result = 0;
	if(dst)
	{
		result = size;
		while(size--)
			*dst++ = *src++;
	}
	return result;
}

// Get the uncompressed size of a buffer containing compressed SBZ data.
// Return a negative value if it appears invalid.
static inline enum sbz_status decode(const uint8_t *input_buf, uint32_t input_size, uint32_t *output_size, uint8_t *output_data)
{
	const uint8_t *desc_head = input_buf, *input_head = input_buf, *input_tail = input_buf + input_size;

	uint8_t bits_left = 0;
	uint16_t desc_word;
	uint32_t cmd = -1;
	uint32_t tmp;

	uint8_t eof = 0;

	if(!output_size)
		output_size = &tmp;

	*output_size = 0;

	if((desc_head + 1) < input_tail)
	{
		uint16_t skip = (*desc_head++) << 8;
		skip += (*desc_head++) & 0xff;
		input_head = desc_head + skip;
	}

	while(input_head < input_tail && desc_head < input_tail)
	{
		if(!bits_left--)
		{
			if(input_head >= input_tail)
				return SBZ_OVERFLOW;
			desc_word = (*desc_head++) << 8;

			if(input_head >= input_tail)
				return SBZ_OVERFLOW;
			desc_word |= (*desc_head++);

			bits_left = 15;
		}

		if(desc_word & 0x8000)
		{
			desc_word <<= 1;
			if(!bits_left--)
			{
				if(input_head >= input_tail)
					return SBZ_OVERFLOW;
				desc_word = (*desc_head++) << 8;

				if(input_head >= input_tail)
					return SBZ_OVERFLOW;
				desc_word |= (*desc_head++);

				bits_left = 15;
			}

			int16_t copy_length = 0;
			int16_t copy_offset = -1;

			cmd = *input_head++;
			if(desc_word & 0x8000)
			{
				if(cmd & 0x80)
				{
					if(cmd < 0xc0) // Extended copy
					{
						uint16_t temp = (desc_head[0] << 8) | desc_head[1];
						desc_head += 2;
						copy_offset = (temp >> 2) & 0x3fff;
						copy_length = max_copy_e_length - (((cmd << 2) ^ temp) & 0xff);
					}
					else if(cmd != 0xc0) // Direct command
					{
						copy_offset = -1;
						copy_length = 1 + max_direct_length - (cmd & 0x3f);
					}
					else // End
					{
						eof = 1;
						break;
					}
				}
				else // Long copy
				{
					if(input_head >= input_tail)
						return SBZ_OVERFLOW;
					copy_offset = ((cmd << 3) & 0x300) | ((*input_head++) & 0xff);
					copy_length = max_copy_l_length - (cmd & 0x1f);
				}
			}
			else // Short copy
			{
				copy_length = max_copy_s_length - (cmd & 0x07);
				copy_offset = max_copy_s_offset - ((cmd >> 3) & 0x1f);
			}

			if(copy_offset == -1)
			{
				// Copy directly from the buffer
				// Length of copy may not exceed the buffer size
				if(input_head + copy_length > input_tail)
					return SBZ_OVERFLOW;
				output_data += my_memcpy(output_data, input_head, copy_length);
				input_head += copy_length;
			}
			else
			{
				// Copy from the sliding window
				// Offset may not be out of bounds
				if((*output_size - copy_offset < 0) || (copy_offset == 0))
					return SBZ_INVALID_OFFSET;
				output_data += my_memcpy(output_data, output_data - copy_offset, copy_length);
			}
			*output_size += copy_length;
		}
		else
		{
			output_data += my_memcpy(output_data, input_head, 1);
			input_head++;
			(*output_size)++;
		}

		desc_word <<= 1;

		// The output data appears too large
		if(*output_size > MAX_SBZ_SIZE)
			return SBZ_INVALID_LENGTH;
	}

	// There's still data left but we can't decompress it.
	if(input_head < input_tail)
		return 0; //return SBZ_UNDERFLOW;

	// There is no EOF marker at the end
	if(!eof)
		return SBZ_INCOMPLETE;

	return SBZ_OK;
}

// Perform decompression. Return value < 0 indicates abort.
enum sbz_status sbz_decompress(const uint8_t *in_buf, uint32_t in_size, uint8_t **out_buf, uint32_t *out_size)
{
	enum sbz_status status;
	uint32_t size;

	status = decode(in_buf, in_size, &size, NULL);
	if(status >= 0)
	{
		uint8_t *buffer;
		status = SBZ_ERROR;
		buffer = (uint8_t*)malloc(size);
		if(buffer)
		{
			status = decode(in_buf, in_size, &size, buffer);
			*out_buf = buffer;
			*out_size = size;
		}
	}

	return status;
}

enum sbz_status sbz_verify(const uint8_t *in_buf, uint32_t in_size, const uint8_t *sbz_buf, uint32_t sbz_size)
{
	enum sbz_status status = SBZ_ERROR;

	uint8_t *dec_buf;
	uint32_t dec_size;

	status = sbz_decompress(sbz_buf, sbz_size, &dec_buf, &dec_size);
	if(status == SBZ_OK)
	{
		status = SBZ_VERIFY_ERROR;
		if(dec_size == in_size)
		{
			if(!memcmp(in_buf, dec_buf, dec_size))
				status = SBZ_OK;
		}
		free(dec_buf);
	}
	return status;
}

const char* sbz_get_decompress_message(enum sbz_status status)
{
	switch(status)
	{
		case SBZ_ERROR:
			return "Memory allocation error";
		case SBZ_UNDERFLOW:
			return "input data remaining";
		case SBZ_OVERFLOW:
			return "incomplete token";
		case SBZ_INCOMPLETE:
			return "no EOF token";
		case SBZ_INVALID_LENGTH:
			return "invalid length";
		case SBZ_INVALID_OFFSET:
			return "invalid offset";
		case SBZ_VERIFY_ERROR:
			return "verify error";
		case SBZ_OK:
			return "ok";
		default:
			return "unknown";
	}
}

#endif // SBZ_LIB_INCLUDED
#endif // SBZ_HEADER_ONLY

// ====================================================================
// Beginning of SBZ tool source code
// ====================================================================
#ifdef SBZ_TOOL

#include <ctype.h>

#define APP_TITLE "SBZ" // Compression title
#define APP_VERSION "3.0"
#define APP_EXT "sbz" // File extension
#define COMPRESS(a,b,c,d,e) sbz_compress(a,b,c,d)
#define DECOMPRESS(a,b,c,d,e) sbz_decompress(a,b,c,d)
#define VERIFY(a,b,c,d,e) sbz_verify(a,b,c,d)
#define GET_MSG sbz_get_decompress_message
#define DUMP_NODES(a,b,c,d) sbz_dump_nodes(a,b,c)
#define ENUM sbz_status
#define ERROR SBZ_ERROR
#define OK SBZ_OK

static int read_file(const char* filename, uint8_t** dataptr, uint32_t* filesize)
{
	FILE* sourcefile;
	sourcefile = fopen(filename,"rb");

	if(!sourcefile)
	{
		printf("Could not open %s\n",filename);
		perror("Error");
		return -1;
	}
	fseek(sourcefile,0,SEEK_END);
	*filesize = ftell(sourcefile);
	rewind(sourcefile);
	*dataptr = (uint8_t*)malloc((*filesize)+128);

	int32_t res = fread(*dataptr,1,*filesize,sourcefile);
	if(res != *filesize)
	{
		printf("Reading error\n");
		return -1;
	}
	fclose(sourcefile);

	return 0;
}

static int write_file(const char* filename, uint8_t* dataptr, uint32_t datasize)
{
	FILE *destfile;

	destfile = fopen(filename,"wb");
	if(!destfile)
	{
		printf("Could not open %s\n",filename);
		return -1;
	}
	int32_t res = fwrite(dataptr, 1, datasize, destfile);
	if(res != datasize)
	{
		printf("Writing error\n");
		return -1;
	}
	fclose(destfile);
	return 0;
}

int main(int argc, char *argv[])
{
	enum ENUM status = ERROR;

	uint8_t *input_data, *output_data;
	uint32_t input_size, output_size;

	const char *input_fn = "", *output_fn = "";
	int has_input = 0, has_output = 0;
	int mode = 0;
	__attribute__((unused)) int delta = 1;

	for(int i = 1; i < argc; i++)
	{
		if(!strcmp(argv[i], "-c"))
			mode = 0;
		else if(!strcmp(argv[i], "-d"))
			mode = 1;
		else if(!strcmp(argv[i], "-dump"))
			mode = 2;
		else if(!strcmp(argv[i], "-h"))
			mode = -1;
		else if(!has_input)
			has_input = 1, input_fn = argv[i];
		else if(!has_output)
			has_output = 1, output_fn = argv[i];
	}

	if(!has_input || mode == -1)
	{
		printf(APP_TITLE " compressor / decompressor by ctr\n");
		printf("Version " APP_VERSION "\n");
		printf("Copyright (c) 2021-2024 Ian Karlsson\n");
		printf("Usage:\n");
		printf("\tTo compress: %s input.bin <output." APP_EXT ">\n", argv[0]);
		printf("\tTo decompress: %s -d input." APP_EXT " <output.dec>\n", argv[0]);
		exit(-1);
	}
	else
	{
		if(!has_output)
		{
			static char temp_fn[FILENAME_MAX];
			strcpy(temp_fn, input_fn);
			if(mode == 0)
				strcat(temp_fn, "." APP_EXT);
			else if(mode == 1)
				strcat(temp_fn, ".dec");
			else if(mode == 2)
				strcat(temp_fn, ".txt");
			output_fn = temp_fn;
		}
	}

	if(!read_file(input_fn, &input_data, &input_size))
	{
		if(mode == 2)
		{
			FILE* output = fopen(output_fn, "w");
			status = DUMP_NODES(input_data, input_size, output, delta);
			fclose(output);
		}
		else
		{
			if(mode == 0)
			{
				status = COMPRESS(input_data, input_size, &output_data, &output_size, delta);
				if(status == OK)
					status = VERIFY(input_data, input_size, output_data, output_size, delta);
			}
			else
			{
				status = DECOMPRESS(input_data, input_size, &output_data, &output_size, delta);
			}

			if(status < OK)
				printf("Fatal error: %s\n", GET_MSG(status));
			else if(status != OK)
				printf("Warning: %s\n", GET_MSG(status));

			if(status >= OK)
			{
				double ratio = (double)output_size / input_size;
				printf("%d -> %d (Ratio: %.2f)\n", input_size, output_size, ratio);
				if(write_file(output_fn, output_data, output_size))
				{
					status = ERROR;
				}
			}
		}
	}

	if(status < OK)
		return -1;
	else
		return 0;
}

#endif // SBZ_TOOL
// vim: set sw=4 ts=4 noet:
