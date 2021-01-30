/*
 * Computer Architecture - Professor Onur Mutlu
 *
 * MIPS pipeline timing simulator
 *
 * Chris Fallin, 2012
 */

#ifndef _PIPE_H_
#define _PIPE_H_

#include "shell.h"


#define ICACHE_NUM_SETS 64
#define ICACHE_ASSOCIATIVITY 4 //Number of blocks per set

typedef struct instr_cache_block {

    uint32_t instruction;
    uint32_t address;
    int valid;
    int lru; //Max value for this would be equal to ICACHE_ASSOCIATIVITY

} icache_block;

// External instruction cache
extern icache_block Icache[ICACHE_NUM_SETS][ICACHE_ASSOCIATIVITY]; //Each element of this would be indexed according to appropriate set index


//Data cache
#define DCACHE_NUM_SETS 256
#define DCACHE_ASSOCIATIVITY 8 //Number of blocks per set

typedef struct data_cache_block {

    uint32_t data;
    uint32_t address;
    int valid;
    int lru; //Max value for this would be equal to DCACHE_ASSOCIATIVITY

} dcache_block;

// External data cache
extern dcache_block Dcache[DCACHE_NUM_SETS][DCACHE_ASSOCIATIVITY]; //Each element of this would be indexed according to appropriate set index

// L2 Unified Cache
#define L2CACHE_NUM_SETS 512
#define L2CACHE_ASSOCIATIVITY 16 //Number of blocks per set

typedef struct L2_cache_block {

    uint32_t data;
    uint32_t address;
    int valid;
    int lru; //Max value for this would be equal to DCACHE_ASSOCIATIVITY

} L2cache_block;

// External data cache
extern L2cache_block L2cache[L2CACHE_NUM_SETS][L2CACHE_ASSOCIATIVITY]; //Each element of this would be indexed according to appropriate set index

#define L2__HIT_LATENCY 15 
#define L2_MISS_CONTROLLER_LATENCY 5 //Latency between L2 cache and memory controller
#define L2_MISS__LATENCY 5

// DRAM parameters

#define DRAM_NUM_BANKS 8
#define DRAM_NUM_ROWS 65536
#define DRAM_REQUEST_QUEUE_SIZE 1000

//DRAM timing parameters
#define DRAM_ACTIVATE_LATENCY 4
#define DRAM_PRECHARGE_LATENCY 4
#define DRAM_READWRITE_LATENCY 4
#define DRAM_BANK_BUSY_LATENCY 100
#define DRAM_DATA_BUS_LATENCY 50


typedef struct request {
    uint32_t address;
    int ready;
    uint32_t arrival; // Request arrival cycle. Set to stat_cycle

} Request;

extern Request DRAMRequestQueue[DRAM_REQUEST_QUEUE_SIZE];
extern int DRAMOpenedRow[DRAM_NUM_BANKS]; // Track the opened row in each bank
extern int DRAMRequestQueueLastIdx;

/* Pipeline ops (instances of this structure) are high-level representations of
 * the instructions that actually flow through the pipeline. This struct does
 * not correspond 1-to-1 with the control signals that would actually pass
 * through the pipeline. Rather, it carries the original instruction, operand
 * information and values as they are collected, and destination information. */
typedef struct Pipe_Op {
    /* PC of this instruction */
    uint32_t pc;
    /* raw instruction */
    uint32_t instruction;
    /* decoded opcode and subopcode fields */
    int opcode, subop;

    /* immediate value, if any, for ALU immediates */
    uint32_t imm16, se_imm16;
    /* shift amount */
    int shamt;

    /* register source values */
    int reg_src1, reg_src2; /* 0 -- 31 if this inst has register source(s), or
                               -1 otherwise */
    uint32_t reg_src1_value, reg_src2_value; /* values of operands from source
                                                regs */

    /* memory access information */
    int is_mem;       /* is this a load/store? */
    uint32_t mem_addr; /* address if applicable */
    int mem_write; /* is this a write to memory? */
    uint32_t mem_value; /* value loaded from memory or to be written to memory */

    /* register destination information */
    int reg_dst; /* 0 -- 31 if this inst has a destination register, -1
                    otherwise */
    uint32_t reg_dst_value; /* value to write into dest reg. */
    int reg_dst_value_ready; /* destination value produced yet? */

    /* branch information */
    int is_branch;        /* is this a branch? */
    uint32_t branch_dest; /* branch destination (if taken) */
    int branch_cond;      /* is this a conditional branch? */
    int branch_taken;     /* branch taken? (set as soon as resolved: in decode
                             for unconditional, execute for conditional) */
    int is_link;          /* jump-and-link or branch-and-link inst? */
    int link_reg;         /* register to place link into? */


} Pipe_Op;

/* The pipe state represents the current state of the pipeline. It holds a
 * pointer to the op that is currently at the input of each stage. As stages
 * execute, they remove the op from their input (set the pointer to NULL) and
 * place an op at their output. If the pointer that represents a stage's output
 * is not null when that stage executes, then this represents a pipeline stall,
 * and the stage must not overwrite its output (otherwise an instruction would
 * be lost).
 */

typedef struct Pipe_State {
    /* pipe op currently at the input of the given stage (NULL for none) */
    Pipe_Op *decode_op, *execute_op, *mem_op, *wb_op;

    /* register file state */
    uint32_t REGS[32];
    uint32_t HI, LO;

    /* program counter in fetch stage */
    uint32_t PC;

    /* information for PC update (branch recovery). Branches should use this
     * mechanism to redirect the fetch stage, and flush the ops that came after
     * the branch as necessary. */
    int branch_recover; /* set to '1' to load a new PC */
    uint32_t branch_dest; /* next fetch will be from this PC */
    int branch_flush; /* how many stages to flush during recover? (1 = fetch, 2 = fetch/decode, ...) */

    /* multiplier stall info */
    int multiplier_stall; /* number of remaining cycles until HI/LO are ready */

    /* place other information here as necessary */
    int instr_miss_stall;
    int data_miss_stall;
    
    uint32_t DRAM_Bank_Row_Open_Idx[DRAM_NUM_BANKS];

} Pipe_State;

/* global variable -- pipeline state */
extern Pipe_State pipe;

/* called during simulator startup */
void pipe_init();

/* this function calls the others */
void pipe_cycle();

/* helper: pipe stages can call this to schedule a branch recovery */
/* flushes 'flush' stages (1 = execute only, 2 = fetch/decode, ...) and then
 * sets the fetch PC to the given destination. */
void pipe_recover(int flush, uint32_t dest);

/* each of these functions implements one stage of the pipeline */
void pipe_stage_fetch();
void pipe_stage_decode();
void pipe_stage_execute();
void pipe_stage_mem();
void pipe_stage_wb();

#endif
