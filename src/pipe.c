/*
 * Computer Architecture - Professor Onur Mutlu
 *
 * MIPS pipeline timing simulator
 *
 * Chris Fallin, 2012
 */

#include "pipe.h"
#include "shell.h"
#include "mips.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

//#define DEBUG

/* debug */
void print_op(Pipe_Op *op)
{
    if (op)
        printf("OP (PC=%08x inst=%08x) src1=R%d (%08x) src2=R%d (%08x) dst=R%d valid %d (%08x) br=%d taken=%d dest=%08x mem=%d addr=%08x\n",
                op->pc, op->instruction, op->reg_src1, op->reg_src1_value, op->reg_src2, op->reg_src2_value, op->reg_dst, op->reg_dst_value_ready,
                op->reg_dst_value, op->is_branch, op->branch_taken, op->branch_dest, op->is_mem, op->mem_addr);
    else
        printf("(null)\n");
}

/* global pipeline state */
Pipe_State pipe;


Request DRAMRequestQueue[DRAM_REQUEST_QUEUE_SIZE];
int DRAMOpenedRow[DRAM_NUM_BANKS]; // Track the opened row in each bank
int DRAMRequestQueueLastIdx;

void DRAM_Insert_to_RequestQueue(uint32_t address) //Call when adding a new request
{
    Request NewRequest;
    NewRequest.address=address;
    NewRequest.ready = 0;
    NewRequest.arrival = stat_cycles;
    
    DRAMRequestQueue[DRAMRequestQueueLastIdx].address = NewRequest.address;
    DRAMRequestQueue[DRAMRequestQueueLastIdx].ready = NewRequest.ready;
    DRAMRequestQueue[DRAMRequestQueueLastIdx].arrival = NewRequest.arrival;

    DRAMRequestQueueLastIdx = DRAMRequestQueueLastIdx + 1;
}

void DRAM_Delete_From_RequestQueue(uint32_t reqaddr) //Call when deleting a new request
{
    int position;
    for(int i=0; i<DRAMRequestQueueLastIdx; i++)
    {
        if(DRAMRequestQueue[DRAMRequestQueueLastIdx].address == reqaddr)
            {
                position = i;
                break;
            }
    }

    for(int c=position -1; c<DRAMRequestQueueLastIdx -1; c++)
    {
        DRAMRequestQueue[c].address = DRAMRequestQueue[c+1].address;
        DRAMRequestQueue[c].ready = DRAMRequestQueue[c+1].ready;
        DRAMRequestQueue[c].arrival = DRAMRequestQueue[c+1].arrival;
    }
    DRAMRequestQueueLastIdx = DRAMRequestQueueLastIdx -1;
}

Request ScheduleRequest()
{
    uint32_t RowIdx;
    uint32_t BankIdx;
    uint32_t MinArrival;

    int ReqIdx, i;

    // Check if request is ready
    for(ReqIdx = 0; ReqIdx < DRAMRequestQueueLastIdx; ReqIdx ++)
    {
        BankIdx = (DRAMRequestQueue[i].address>>5) & 0X00000007; //addr[7:5]
        RowIdx = (DRAMRequestQueue[i].address>>16) & 0X0000FFFF; //addr[31:16]
        if(RowIdx == DRAMOpenedRow[BankIdx])
            break;
    }

    // Find the request with the least arrival time
    MinArrival = DRAMRequestQueue[0].arrival;
    if(ReqIdx == DRAMRequestQueueLastIdx)
    {
        for(i = 1; i<DRAMRequestQueueLastIdx; i++)
        {
            if(MinArrival < DRAMRequestQueue[i].arrival)
                ReqIdx = i;
        }       
        pipe.DRAM_commandbus_stall = 12; //Precharge + Activate + Read
    }
    else
    {
        pipe.DRAM_commandbus_stall = 4; //Read command only   
    }

    pipe.DRAM_Bank_Busy_stall[BankIdx] = 100;
    return DRAMRequestQueue[ReqIdx];
}

void DRAM_Bank_Busy() //To model bank busy utilization. Will be called per cycle
{
    int i;
    for(i = 0; i< DRAM_NUM_BANKS ; i++)
    {
        if(pipe.DRAM_Bank_Busy_stall[i]>0)
        {
                pipe.DRAM_Bank_Busy_stall[i]--;
        }
        else
        {
            if(pipe.DRAMDataBusFree)
            {   
                pipe.DRAM_databus_stall = 100;
                pipe.DRAM_databus_addr = pipe.DRAM_Bank_Req_addr[i];
                DRAM_Delete_From_RequestQueue(pipe.DRAM_Bank_Req_addr[i]);
                pipe.DRAMDataBusFree = 0;
                break;
            }
        }
    }
}

void DRAM_Send_Command() //To model Send Command utilization
{
    if(pipe.DRAM_commandbus_stall > 0)
    {
        pipe.DRAM_commandbus_stall--;
        
    }
    ScheduleRequest();
    if(pipe.DRAM_commandbus_stall > 0)
    {
        pipe.DRAM_commandbus_stall--;
        return;
    }
}

void DRAM_Send_Data() //To model data bus utilization
{
    if(pipe.DRAM_databus_stall > 0)
    {
        pipe.DRAM_databus_stall--;
    }
    
    // Some code which would notify L2 that data has now been transferred to the bus
    // Remove from MSHR entry now

    if(pipe.DRAM_databus_stall > 0)
    {
        return;
    }
    pipe.DRAM_L2_databus_addr = pipe.DRAM_databus_addr;
    pipe.DRAMDataBusFree = 1;
}


void DRAM_Memory_Controller()
{
    //DRAM_Insert_to_RequestQueue(NewRequest); L2 cache should call this function.
    // Check timing constraints
    // MSHR operations
    
    ScheduleRequest();

}

mshr L2MSHR[NUM_MSHR];

void AllocateMSHR(uint32_t address)
{
    int i;
    for(i=0;i<NUM_MSHR;i++)
    {
        if(L2MSHR[i].valid==0)
            break;
    }

    //TODO: Add check if NUM_MSHR has been exceeded.
    //TODO: Initialize MSHR Array
    L2MSHR[i].valid = 1;
    L2MSHR[i].address = address;
    L2MSHR[i].done = 0;
}

void DeallocateMSHR(uint32_t address)
{
    int i;
    for(i=0;i<NUM_MSHR;i++)
    {
        if(L2MSHR[i].address==address)
            break;
    }

    L2MSHR[i].valid = 0;
    L2MSHR[i].address = address;
    L2MSHR[i].done = 1;
}

void MSHRtoController()
{
    if(pipe.L2_miss_mem_addr==-1)
        return;

    if(pipe.L2_memory_ctrl_stall>0)
    {   pipe.L2_memory_ctrl_stall--;
        return;
    }
    
    DRAM_Insert_to_RequestQueue(pipe.L2_miss_mem_addr);
}

/*L2 cache */
L2cache_block L2cache[L2CACHE_NUM_SETS][L2CACHE_ASSOCIATIVITY];

//req_type is to know if request is from instruction cache or data cache.
uint32_t L2cache_lookup(uint32_t mem_addr, int req_type)
{
    uint32_t set_index = (mem_addr>>5)&(0X000001FF); //Set index = PC[13:5]
    int blockIdx;

    for(blockIdx = 0; blockIdx< L2CACHE_ASSOCIATIVITY; blockIdx++)
    {
        if((mem_addr == L2cache[set_index][blockIdx].address)&& (L2cache[set_index][blockIdx].valid ==1))
            break; //Its a hit
    }

    // Its a miss
    if(blockIdx == L2CACHE_ASSOCIATIVITY)
    {
        for(blockIdx = 0; blockIdx< L2CACHE_ASSOCIATIVITY; blockIdx++)
            {
                if(L2cache[set_index][blockIdx].lru == L2CACHE_ASSOCIATIVITY-1)
                    break; //Detected the lru block which can be replaced
            }
        
        AllocateMSHR(mem_addr);
        pipe.L2_memory_ctrl_stall = 5;
        pipe.L2_miss_mem_addr = mem_addr;        
        L2cache[set_index][blockIdx].address = mem_addr;
        L2cache[set_index][blockIdx].data = mem_read_32(mem_addr);
        
        // Once request returned. remove from MSHR
        DeallocateMSHR(mem_addr);

        L2cache[set_index][blockIdx].valid = 1;

        //Update LRU status
        for(int i = 0; i< L2CACHE_ASSOCIATIVITY; i++)
            {
                if( L2cache[set_index][i].lru < L2cache[set_index][blockIdx].lru)
                    L2cache[set_index][i].lru++;                
            }
    }
    else
    {
        //Stall due to L2 hit
        if(req_type==0) //Instruction
        {
            pipe.instr_miss_stall = 15;
        }
        else //Data
        {
            pipe.data_miss_stall = 15;
        }
        pipe.L2_miss_mem_addr = -1;
        pipe.L2_memory_ctrl_stall = 0;

        for(int i=0; i<L2CACHE_ASSOCIATIVITY; i++)
        {
            if( L2cache[set_index][i].lru < L2cache[set_index][blockIdx].lru)
                L2cache[set_index][i].lru++;
        }
    }
    L2cache[set_index][blockIdx].lru = 0;
    return L2cache[set_index][blockIdx].data;
}


void L2cache_write(uint32_t mem_addr, uint32_t val)
{
    uint32_t set_index = (mem_addr>>5)&(0X000000FF); //Set index = PC[12:5]

    int blockIdx;
    for(blockIdx = 0; blockIdx< L2CACHE_ASSOCIATIVITY; blockIdx++)
    {
        if((mem_addr == L2cache[set_index][blockIdx].address)&& (L2cache[set_index][blockIdx].valid ==1))
            break; //Its a hit
    }

    // Its a miss
    if(blockIdx == L2CACHE_ASSOCIATIVITY)
    {
        for(blockIdx = 0; blockIdx< L2CACHE_ASSOCIATIVITY; blockIdx++)
            {
                if(L2cache[set_index][blockIdx].lru == L2CACHE_ASSOCIATIVITY-1)
                    break; //Detected the lru block which can be replaced
            }
        
        //Update LRU status
        for(int i = 0; i< L2CACHE_ASSOCIATIVITY; i++)
            {
                if( L2cache[set_index][i].lru < L2cache[set_index][blockIdx].lru)
                    L2cache[set_index][i].lru++;                
            }
    }
    else
    {
        //Stall due to L2 hit
        pipe.data_miss_stall = 15;
        for(int i=0; i<L2CACHE_ASSOCIATIVITY; i++)
        {
            if( L2cache[set_index][i].lru < L2cache[set_index][blockIdx].lru)
                L2cache[set_index][i].lru++;
        }
    }

    AllocateMSHR(mem_addr);
    L2cache[set_index][blockIdx].address = mem_addr;
    mem_write_32(mem_addr, val);
    DeallocateMSHR(mem_addr);
    L2cache[set_index][blockIdx].data = val;
    L2cache[set_index][blockIdx].valid = 1;
    L2cache[set_index][blockIdx].lru = 0;
}


/*Instruction cache */
icache_block Icache[ICACHE_NUM_SETS][ICACHE_ASSOCIATIVITY];

uint32_t icache_lookup(uint32_t mem_addr)
{
    uint32_t set_index = (mem_addr>>5)&(0X0000003F); //Set index = PC[10:5]

    int blockIdx;
    for(blockIdx = 0; blockIdx< ICACHE_ASSOCIATIVITY; blockIdx++)
    {
        if((mem_addr == Icache[set_index][blockIdx].address)&& (Icache[set_index][blockIdx].valid ==1))
            break; //Its a hit
    }

    // Its a miss
    if(blockIdx == ICACHE_ASSOCIATIVITY)
    {

        for(blockIdx = 0; blockIdx< ICACHE_ASSOCIATIVITY; blockIdx++)
            {
                if(Icache[set_index][blockIdx].lru == ICACHE_ASSOCIATIVITY-1)
                    break; //Detected the lru block which can be replaced
            }
        
        Icache[set_index][blockIdx].address = mem_addr;
        Icache[set_index][blockIdx].instruction = L2cache_lookup(mem_addr, 0);
        Icache[set_index][blockIdx].valid = 1;

        //Update LRU status
        for(int i = 0; i< ICACHE_ASSOCIATIVITY; i++)
            {
                if( Icache[set_index][i].lru < Icache[set_index][blockIdx].lru)
                    Icache[set_index][i].lru++;                
            }
    }
    else
    {
        for(int i=0; i<ICACHE_ASSOCIATIVITY; i++)
        {
            if( Icache[set_index][i].lru < Icache[set_index][blockIdx].lru)
                Icache[set_index][i].lru++;
        }
    }
    Icache[set_index][blockIdx].lru = 0;

    return Icache[set_index][blockIdx].instruction;
}

/*Data cache */
dcache_block Dcache[DCACHE_NUM_SETS][DCACHE_ASSOCIATIVITY];

uint32_t dcache_lookup(uint32_t mem_addr)
{
    uint32_t set_index = (mem_addr>>5)&(0X000000FF); //Set index = PC[12:5]
    int blockIdx;

    for(blockIdx = 0; blockIdx< DCACHE_ASSOCIATIVITY; blockIdx++)
    {

        if((mem_addr == Dcache[set_index][blockIdx].address)&& (Dcache[set_index][blockIdx].valid ==1))
            break; //Its a hit
    }

    // Its a miss
    if(blockIdx == DCACHE_ASSOCIATIVITY)
    {
        for(blockIdx = 0; blockIdx< DCACHE_ASSOCIATIVITY; blockIdx++)
            {
                if(Dcache[set_index][blockIdx].lru == DCACHE_ASSOCIATIVITY-1)
                    break; //Detected the lru block which can be replaced
            }
        
        Dcache[set_index][blockIdx].address = mem_addr;
        Dcache[set_index][blockIdx].data = L2cache_lookup(mem_addr, 1);
        Dcache[set_index][blockIdx].valid = 1;

        //Update LRU status
        for(int i = 0; i< DCACHE_ASSOCIATIVITY; i++)
            {
                if( Dcache[set_index][i].lru < Dcache[set_index][blockIdx].lru)
                    Dcache[set_index][i].lru++;                
            }
    }
    else
    {
        for(int i=0; i<DCACHE_ASSOCIATIVITY; i++)
        {
            if( Dcache[set_index][i].lru < Dcache[set_index][blockIdx].lru)
                Dcache[set_index][i].lru++;
        }
    }
    Dcache[set_index][blockIdx].lru = 0;
    return Dcache[set_index][blockIdx].data;
}


void dcache_write(uint32_t mem_addr, uint32_t val)
{
    uint32_t set_index = (mem_addr>>5)&(0X000000FF); //Set index = PC[12:5]

    int blockIdx;
    for(blockIdx = 0; blockIdx< DCACHE_ASSOCIATIVITY; blockIdx++)
    {
        if((mem_addr == Dcache[set_index][blockIdx].address)&& (Dcache[set_index][blockIdx].valid ==1))
            break; //Its a hit
    }

    // Its a miss
    if(blockIdx == DCACHE_ASSOCIATIVITY)
    {
        
        for(blockIdx = 0; blockIdx< DCACHE_ASSOCIATIVITY; blockIdx++)
            {
                if(Dcache[set_index][blockIdx].lru == DCACHE_ASSOCIATIVITY-1)
                    break; //Detected the lru block which can be replaced
            }
        
        //Update LRU status
        for(int i = 0; i< DCACHE_ASSOCIATIVITY; i++)
            {
                if( Dcache[set_index][i].lru < Dcache[set_index][blockIdx].lru)
                    Dcache[set_index][i].lru++;                
            }
    }
    else
    {
        for(int i=0; i<DCACHE_ASSOCIATIVITY; i++)
        {
            if( Dcache[set_index][i].lru < Dcache[set_index][blockIdx].lru)
                Dcache[set_index][i].lru++;
        }
    }

    Dcache[set_index][blockIdx].address = mem_addr;
    L2cache_write(mem_addr, val);

    Dcache[set_index][blockIdx].data = val;
    Dcache[set_index][blockIdx].valid = 1;
    Dcache[set_index][blockIdx].lru = 0;
}


void pipe_init()
{
    memset(&pipe, 0, sizeof(Pipe_State));
    pipe.PC = 0x00400000;

    //Initializing elements of instruction cache
    for(uint32_t set_index=0; set_index<ICACHE_NUM_SETS;set_index++)
        {
            for (int blockIdx=0; blockIdx<ICACHE_ASSOCIATIVITY; blockIdx++)
                {
                    Icache[set_index][blockIdx].lru = blockIdx;
                    Icache[set_index][blockIdx].valid = 0;
                    Icache[set_index][blockIdx].instruction = 0;
                }       
        }   


    //Initializing elements of data cache
    for(uint32_t set_index=0; set_index<DCACHE_NUM_SETS;set_index++)
        {
            for (int blockIdx=0; blockIdx<DCACHE_ASSOCIATIVITY; blockIdx++)
                {
                    Dcache[set_index][blockIdx].lru = blockIdx;
                    Dcache[set_index][blockIdx].valid = 0;
                    Dcache[set_index][blockIdx].data = 0;
                }       
        }        

    pipe.instr_miss_stall = 0;
    pipe.data_miss_stall = 0;

    DRAMRequestQueueLastIdx = 0;
}


void pipe_cycle()
{
#ifdef DEBUG
    printf("\n\n----\n\nPIPELINE:\n");
    printf("DCODE: "); print_op(pipe.decode_op);
    printf("EXEC : "); print_op(pipe.execute_op);
    printf("MEM  : "); print_op(pipe.mem_op);
    printf("WB   : "); print_op(pipe.wb_op);
    printf("\n");
#endif

    MSHRtoController();
    pipe_stage_wb();
    pipe_stage_mem();
    pipe_stage_execute();
    pipe_stage_decode();
    pipe_stage_fetch();

    /* handle branch recoveries */
    if (pipe.branch_recover) {
#ifdef DEBUG
        printf("branch recovery: new dest %08x flush %d stages\n", pipe.branch_dest, pipe.branch_flush);
#endif

        pipe.PC = pipe.branch_dest;

        if (pipe.branch_flush >= 2) {
            if (pipe.decode_op) free(pipe.decode_op);
            pipe.decode_op = NULL;
        }

        if (pipe.branch_flush >= 3) {
            if (pipe.execute_op) free(pipe.execute_op);
            pipe.execute_op = NULL;
        }

        if (pipe.branch_flush >= 4) {
            if (pipe.mem_op) free(pipe.mem_op);
            pipe.mem_op = NULL;
        }

        if (pipe.branch_flush >= 5) {
            if (pipe.wb_op) free(pipe.wb_op);
            pipe.wb_op = NULL;
        }

        pipe.branch_recover = 0;
        pipe.branch_dest = 0;
        pipe.branch_flush = 0;

        stat_squash++;
    }
}

void pipe_recover(int flush, uint32_t dest)
{
    /* if there is already a recovery scheduled, it must have come from a later
     * stage (which executes older instructions), hence that recovery overrides
     * our recovery. Simply return in this case. */
    if (pipe.branch_recover) return;

    /* schedule the recovery. This will be done once all pipeline stages simulate the current cycle. */
    pipe.branch_recover = 1;
    pipe.branch_flush = flush;
    pipe.branch_dest = dest;
}

void pipe_stage_wb()
{
    /* if there is no instruction in this pipeline stage, we are done */
    if (!pipe.wb_op)
        return;

    /* grab the op out of our input slot */
    Pipe_Op *op = pipe.wb_op;
    pipe.wb_op = NULL;

    /* if this instruction writes a register, do so now */
    if (op->reg_dst != -1 && op->reg_dst != 0) {
        pipe.REGS[op->reg_dst] = op->reg_dst_value;
#ifdef DEBUG
        printf("R%d = %08x\n", op->reg_dst, op->reg_dst_value);
#endif
    }

    /* if this was a syscall, perform action */
    if (op->opcode == OP_SPECIAL && op->subop == SUBOP_SYSCALL) {
        if (op->reg_src1_value == 0xA) {
            pipe.PC = op->pc; /* fetch will do pc += 4, then we stop with correct PC */
            RUN_BIT = 0;
        }
    }

    /* free the op */
    free(op);
    stat_inst_retire++;
}

void pipe_stage_mem()
{
    
    if (pipe.data_miss_stall > 0)
    {
        pipe.data_miss_stall--;
        return;
    }

    /* if there is no instruction in this pipeline stage, we are done */
    if (!pipe.mem_op)
        return;

    /* grab the op out of our input slot */
    Pipe_Op *op = pipe.mem_op;

    uint32_t val = 0;
    if (op->is_mem)
        {
        val = dcache_lookup(op->mem_addr& ~3);
        }
    switch (op->opcode) {
        case OP_LW:
        case OP_LH:
        case OP_LHU:
        case OP_LB:
        case OP_LBU:
            {
                /* extract needed value */
                op->reg_dst_value_ready = 1;
                if (op->opcode == OP_LW) {
                    op->reg_dst_value = val;
                }
                else if (op->opcode == OP_LH || op->opcode == OP_LHU) {
                    if (op->mem_addr & 2)
                        val = (val >> 16) & 0xFFFF;
                    else
                        val = val & 0xFFFF;

                    if (op->opcode == OP_LH)
                        val |= (val & 0x8000) ? 0xFFFF8000 : 0;

                    op->reg_dst_value = val;
                }
                else if (op->opcode == OP_LB || op->opcode == OP_LBU) {
                    switch (op->mem_addr & 3) {
                        case 0:
                            val = val & 0xFF;
                            break;
                        case 1:
                            val = (val >> 8) & 0xFF;
                            break;
                        case 2:
                            val = (val >> 16) & 0xFF;
                            break;
                        case 3:
                            val = (val >> 24) & 0xFF;
                            break;
                    }

                    if (op->opcode == OP_LB)
                        val |= (val & 0x80) ? 0xFFFFFF80 : 0;

                    op->reg_dst_value = val;
                }
            }
            break;

        case OP_SB:
            switch (op->mem_addr & 3) {
                case 0: val = (val & 0xFFFFFF00) | ((op->mem_value & 0xFF) << 0); break;
                case 1: val = (val & 0xFFFF00FF) | ((op->mem_value & 0xFF) << 8); break;
                case 2: val = (val & 0xFF00FFFF) | ((op->mem_value & 0xFF) << 16); break;
                case 3: val = (val & 0x00FFFFFF) | ((op->mem_value & 0xFF) << 24); break;
            }

            dcache_write(op->mem_addr & ~3, val);
            break;

        case OP_SH:
#ifdef DEBUG
            printf("SH: addr %08x val %04x old word %08x\n", op->mem_addr, op->mem_value & 0xFFFF, val);
#endif
            if (op->mem_addr & 2)
                val = (val & 0x0000FFFF) | (op->mem_value) << 16;
            else
                val = (val & 0xFFFF0000) | (op->mem_value & 0xFFFF);
#ifdef DEBUG
            printf("new word %08x\n", val);
#endif
            
            dcache_write(op->mem_addr& ~3, val);
            break;

        case OP_SW:
            val = op->mem_value;
            dcache_write(op->mem_addr& ~3, val);
            break;
    }

    /* clear stage input and transfer to next stage */
    pipe.mem_op = NULL;
    pipe.wb_op = op;
}

void pipe_stage_execute()
{
    /* if a multiply/divide is in progress, decrement cycles until value is ready */
    if (pipe.multiplier_stall > 0)
        pipe.multiplier_stall--;

    /* if downstream stall, return (and leave any input we had) */
    if (pipe.mem_op != NULL)
        return;

    /* if no op to execute, return */
    if (pipe.execute_op == NULL)
        return;

    /* grab op and read sources */
    Pipe_Op *op = pipe.execute_op;

    /* read register values, and check for bypass; stall if necessary */
    int stall = 0;
    if (op->reg_src1 != -1) {
        if (op->reg_src1 == 0)
            op->reg_src1_value = 0;
        else if (pipe.mem_op && pipe.mem_op->reg_dst == op->reg_src1) {
            if (!pipe.mem_op->reg_dst_value_ready)
                stall = 1;
            else
                op->reg_src1_value = pipe.mem_op->reg_dst_value;
        }
        else if (pipe.wb_op && pipe.wb_op->reg_dst == op->reg_src1) {
            op->reg_src1_value = pipe.wb_op->reg_dst_value;
        }
        else
            op->reg_src1_value = pipe.REGS[op->reg_src1];
    }
    if (op->reg_src2 != -1) {
        if (op->reg_src2 == 0)
            op->reg_src2_value = 0;
        else if (pipe.mem_op && pipe.mem_op->reg_dst == op->reg_src2) {
            if (!pipe.mem_op->reg_dst_value_ready)
                stall = 1;
            else
                op->reg_src2_value = pipe.mem_op->reg_dst_value;
        }
        else if (pipe.wb_op && pipe.wb_op->reg_dst == op->reg_src2) {
            op->reg_src2_value = pipe.wb_op->reg_dst_value;
        }
        else
            op->reg_src2_value = pipe.REGS[op->reg_src2];
    }

    /* if bypassing requires a stall (e.g. use immediately after load),
     * return without clearing stage input */
    if (stall) 
        return;

    /* execute the op */
    switch (op->opcode) {
        case OP_SPECIAL:
            op->reg_dst_value_ready = 1;
            switch (op->subop) {
                case SUBOP_SLL:
                    op->reg_dst_value = op->reg_src2_value << op->shamt;
                    break;
                case SUBOP_SLLV:
                    op->reg_dst_value = op->reg_src2_value << op->reg_src1_value;
                    break;
                case SUBOP_SRL:
                    op->reg_dst_value = op->reg_src2_value >> op->shamt;
                    break;
                case SUBOP_SRLV:
                    op->reg_dst_value = op->reg_src2_value >> op->reg_src1_value;
                    break;
                case SUBOP_SRA:
                    op->reg_dst_value = (int32_t)op->reg_src2_value >> op->shamt;
                    break;
                case SUBOP_SRAV:
                    op->reg_dst_value = (int32_t)op->reg_src2_value >> op->reg_src1_value;
                    break;
                case SUBOP_JR:
                case SUBOP_JALR:
                    op->reg_dst_value = op->pc + 4;
                    op->branch_dest = op->reg_src1_value;
                    op->branch_taken = 1;
                    break;

                case SUBOP_MULT:
                    {
                        /* we set a result value right away; however, we will
                         * model a stall if the program tries to read the value
                         * before it's ready (or overwrite HI/LO). Also, if
                         * another multiply comes down the pipe later, it will
                         * update the values and re-set the stall cycle count
                         * for a new operation.
                         */
                        int64_t val = (int64_t)((int32_t)op->reg_src1_value) * (int64_t)((int32_t)op->reg_src2_value);
                        uint64_t uval = (uint64_t)val;
                        pipe.HI = (uval >> 32) & 0xFFFFFFFF;
                        pipe.LO = (uval >>  0) & 0xFFFFFFFF;

                        /* four-cycle multiplier latency */
                        pipe.multiplier_stall = 4;
                    }
                    break;
                case SUBOP_MULTU:
                    {
                        uint64_t val = (uint64_t)op->reg_src1_value * (uint64_t)op->reg_src2_value;
                        pipe.HI = (val >> 32) & 0xFFFFFFFF;
                        pipe.LO = (val >>  0) & 0xFFFFFFFF;

                        /* four-cycle multiplier latency */
                        pipe.multiplier_stall = 4;
                    }
                    break;

                case SUBOP_DIV:
                    if (op->reg_src2_value != 0) {

                        int32_t val1 = (int32_t)op->reg_src1_value;
                        int32_t val2 = (int32_t)op->reg_src2_value;
                        int32_t div, mod;

                        div = val1 / val2;
                        mod = val1 % val2;

                        pipe.LO = div;
                        pipe.HI = mod;
                    } else {
                        // really this would be a div-by-0 exception
                        pipe.HI = pipe.LO = 0;
                    }

                    /* 32-cycle divider latency */
                    pipe.multiplier_stall = 32;
                    break;

                case SUBOP_DIVU:
                    if (op->reg_src2_value != 0) {
                        pipe.HI = (uint32_t)op->reg_src1_value % (uint32_t)op->reg_src2_value;
                        pipe.LO = (uint32_t)op->reg_src1_value / (uint32_t)op->reg_src2_value;
                    } else {
                        /* really this would be a div-by-0 exception */
                        pipe.HI = pipe.LO = 0;
                    }

                    /* 32-cycle divider latency */
                    pipe.multiplier_stall = 32;
                    break;

                case SUBOP_MFHI:
                    /* stall until value is ready */
                    if (pipe.multiplier_stall > 0)
                        return;

                    op->reg_dst_value = pipe.HI;
                    break;
                case SUBOP_MTHI:
                    /* stall to respect WAW dependence */
                    if (pipe.multiplier_stall > 0)
                        return;

                    pipe.HI = op->reg_src1_value;
                    break;

                case SUBOP_MFLO:
                    /* stall until value is ready */
                    if (pipe.multiplier_stall > 0)
                        return;

                    op->reg_dst_value = pipe.LO;
                    break;
                case SUBOP_MTLO:
                    /* stall to respect WAW dependence */
                    if (pipe.multiplier_stall > 0)
                        return;

                    pipe.LO = op->reg_src1_value;
                    break;

                case SUBOP_ADD:
                case SUBOP_ADDU:
                    op->reg_dst_value = op->reg_src1_value + op->reg_src2_value;
                    break;
                case SUBOP_SUB:
                case SUBOP_SUBU:
                    op->reg_dst_value = op->reg_src1_value - op->reg_src2_value;
                    break;
                case SUBOP_AND:
                    op->reg_dst_value = op->reg_src1_value & op->reg_src2_value;
                    break;
                case SUBOP_OR:
                    op->reg_dst_value = op->reg_src1_value | op->reg_src2_value;
                    break;
                case SUBOP_NOR:
                    op->reg_dst_value = ~(op->reg_src1_value | op->reg_src2_value);
                    break;
                case SUBOP_XOR:
                    op->reg_dst_value = op->reg_src1_value ^ op->reg_src2_value;
                    break;
                case SUBOP_SLT:
                    op->reg_dst_value = ((int32_t)op->reg_src1_value <
                            (int32_t)op->reg_src2_value) ? 1 : 0;
                    break;
                case SUBOP_SLTU:
                    op->reg_dst_value = (op->reg_src1_value < op->reg_src2_value) ? 1 : 0;
                    break;
            }
            break;

        case OP_BRSPEC:
            switch (op->subop) {
                case BROP_BLTZ:
                case BROP_BLTZAL:
                    if ((int32_t)op->reg_src1_value < 0) op->branch_taken = 1;
                    break;

                case BROP_BGEZ:
                case BROP_BGEZAL:
                    if ((int32_t)op->reg_src1_value >= 0) op->branch_taken = 1;
                    break;
            }
            break;

        case OP_BEQ:
            if (op->reg_src1_value == op->reg_src2_value) op->branch_taken = 1;
            break;

        case OP_BNE:
            if (op->reg_src1_value != op->reg_src2_value) op->branch_taken = 1;
            break;

        case OP_BLEZ:
            if ((int32_t)op->reg_src1_value <= 0) op->branch_taken = 1;
            break;

        case OP_BGTZ:
            if ((int32_t)op->reg_src1_value > 0) op->branch_taken = 1;
            break;

        case OP_ADDI:
        case OP_ADDIU:
            op->reg_dst_value_ready = 1;
            op->reg_dst_value = op->reg_src1_value + op->se_imm16;
            break;
        case OP_SLTI:
            op->reg_dst_value_ready = 1;
            op->reg_dst_value = (int32_t)op->reg_src1_value < (int32_t)op->se_imm16 ? 1 : 0;
            break;
        case OP_SLTIU:
            op->reg_dst_value_ready = 1;
            op->reg_dst_value = (uint32_t)op->reg_src1_value < (uint32_t)op->se_imm16 ? 1 : 0;
            break;
        case OP_ANDI:
            op->reg_dst_value_ready = 1;
            op->reg_dst_value = op->reg_src1_value & op->imm16;
            break;
        case OP_ORI:
            op->reg_dst_value_ready = 1;
            op->reg_dst_value = op->reg_src1_value | op->imm16;
            break;
        case OP_XORI:
            op->reg_dst_value_ready = 1;
            op->reg_dst_value = op->reg_src1_value ^ op->imm16;
            break;
        case OP_LUI:
            op->reg_dst_value_ready = 1;
            op->reg_dst_value = op->imm16 << 16;
            break;

        case OP_LW:
        case OP_LH:
        case OP_LHU:
        case OP_LB:
        case OP_LBU:
            op->mem_addr = op->reg_src1_value + op->se_imm16;
            break;

        case OP_SW:
        case OP_SH:
        case OP_SB:
            op->mem_addr = op->reg_src1_value + op->se_imm16;
            op->mem_value = op->reg_src2_value;
            break;
    }

    /* handle branch recoveries at this point */
    if (op->branch_taken)
        pipe_recover(3, op->branch_dest);

    /* remove from upstream stage and place in downstream stage */
    pipe.execute_op = NULL;
    pipe.mem_op = op;
}

void pipe_stage_decode()
{
    /* if downstream stall, return (and leave any input we had) */
    if (pipe.execute_op != NULL)
        return;

    /* if no op to decode, return */
    if (pipe.decode_op == NULL)
        return;

    /* grab op and remove from stage input */
    Pipe_Op *op = pipe.decode_op;
    pipe.decode_op = NULL;

    /* set up info fields (source/dest regs, immediate, jump dest) as necessary */
    uint32_t opcode = (op->instruction >> 26) & 0x3F;
    uint32_t rs = (op->instruction >> 21) & 0x1F;
    uint32_t rt = (op->instruction >> 16) & 0x1F;
    uint32_t rd = (op->instruction >> 11) & 0x1F;
    uint32_t shamt = (op->instruction >> 6) & 0x1F;
    uint32_t funct1 = (op->instruction >> 0) & 0x1F;
    uint32_t funct2 = (op->instruction >> 0) & 0x3F;
    uint32_t imm16 = (op->instruction >> 0) & 0xFFFF;
    uint32_t se_imm16 = imm16 | ((imm16 & 0x8000) ? 0xFFFF8000 : 0);
    uint32_t targ = (op->instruction & ((1UL << 26) - 1)) << 2;

    op->opcode = opcode;
    op->imm16 = imm16;
    op->se_imm16 = se_imm16;
    op->shamt = shamt;

    switch (opcode) {
        case OP_SPECIAL:
            /* all "SPECIAL" insts are R-types that use the ALU and both source
             * regs. Set up source regs and immediate value. */
            op->reg_src1 = rs;
            op->reg_src2 = rt;
            op->reg_dst = rd;
            op->subop = funct2;
            if (funct2 == SUBOP_SYSCALL) {
                op->reg_src1 = 2; // v0
                op->reg_src2 = 3; // v1
            }
            if (funct2 == SUBOP_JR || funct2 == SUBOP_JALR) {
                op->is_branch = 1;
                op->branch_cond = 0;
            }

            break;

        case OP_BRSPEC:
            /* branches that have -and-link variants come here */
            op->is_branch = 1;
            op->reg_src1 = rs;
            op->reg_src2 = rt;
            op->is_branch = 1;
            op->branch_cond = 1; /* conditional branch */
            op->branch_dest = op->pc + 4 + (se_imm16 << 2);
            op->subop = rt;
            if (rt == BROP_BLTZAL || rt == BROP_BGEZAL) {
                /* link reg */
                op->reg_dst = 31;
                op->reg_dst_value = op->pc + 4;
                op->reg_dst_value_ready = 1;
            }
            break;

        case OP_JAL:
            op->reg_dst = 31;
            op->reg_dst_value = op->pc + 4;
            op->reg_dst_value_ready = 1;
            op->branch_taken = 1;
            /* fallthrough */
        case OP_J:
            op->is_branch = 1;
            op->branch_cond = 0;
            op->branch_taken = 1;
            op->branch_dest = (op->pc & 0xF0000000) | targ;
            break;

        case OP_BEQ:
        case OP_BNE:
        case OP_BLEZ:
        case OP_BGTZ:
            /* ordinary conditional branches (resolved after execute) */
            op->is_branch = 1;
            op->branch_cond = 1;
            op->branch_dest = op->pc + 4 + (se_imm16 << 2);
            op->reg_src1 = rs;
            op->reg_src2 = rt;
            break;

        case OP_ADDI:
        case OP_ADDIU:
        case OP_SLTI:
        case OP_SLTIU:
            /* I-type ALU ops with sign-extended immediates */
            op->reg_src1 = rs;
            op->reg_dst = rt;
            break;

        case OP_ANDI:
        case OP_ORI:
        case OP_XORI:
        case OP_LUI:
            /* I-type ALU ops with non-sign-extended immediates */
            op->reg_src1 = rs;
            op->reg_dst = rt;
            break;

        case OP_LW:
        case OP_LH:
        case OP_LHU:
        case OP_LB:
        case OP_LBU:
        case OP_SW:
        case OP_SH:
        case OP_SB:
            /* memory ops */
            op->is_mem = 1;
            op->reg_src1 = rs;
            if (opcode == OP_LW || opcode == OP_LH || opcode == OP_LHU || opcode == OP_LB || opcode == OP_LBU) {
                /* load */
                op->mem_write = 0;
                op->reg_dst = rt;
            }
            else {
                /* store */
                op->mem_write = 1;
                op->reg_src2 = rt;
            }
            break;
    }

    /* we will handle reg-read together with bypass in the execute stage */

    /* place op in downstream slot */
    pipe.execute_op = op;
}

void pipe_stage_fetch()
{

    if (pipe.instr_miss_stall > 0)
    {
        pipe.instr_miss_stall--;
    }

    /* if pipeline is stalled (our output slot is not empty), return */    
    if (pipe.decode_op != NULL)
        return;

    /* Allocate an op and send it down the pipeline. */

    Pipe_Op *op = malloc(sizeof(Pipe_Op));
    memset(op, 0, sizeof(Pipe_Op));
    op->reg_src1 = op->reg_src2 = op->reg_dst = -1;

    op->instruction = icache_lookup(pipe.PC);

    if (pipe.instr_miss_stall > 0)
    {
        return;
    }

    op->pc = pipe.PC;
    pipe.decode_op = op;

    /* update PC */
    pipe.PC += 4;

    stat_inst_fetch++;

}
