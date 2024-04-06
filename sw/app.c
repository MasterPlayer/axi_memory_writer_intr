#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xscugic.h"


#define MEMWR_INTR XPAR_FABRIC_MEMWR_USER_EVENT_INTR




typedef struct {
	uint32_t reset_reg;
	uint32_t ctrl_reg;
	uint32_t portion_size_reg;
	uint32_t memory_highaddr_reg;
	uint32_t interval_reg;
	uint32_t freq_hz_reg;
	uint32_t valid_count_reg;
	uint32_t memory_baseaddr_reg;
	uint32_t current_address_reg;
	uint32_t user_event_duration_reg;
	uint32_t transferred_size_reg;
	uint32_t fifo_current_volume;
} axi_memory_writer_intr;



typedef struct {
	uint32_t reset_reg;
	uint32_t ctrl_reg;
	uint32_t packet_size_reg;
	uint32_t packet_limit_reg;
	uint32_t pause_reg;
	uint32_t freq_hz_reg;
	uint32_t valid_count_reg;
	uint32_t width_reg;
	uint32_t data_count_hi_reg;
	uint32_t data_count_lo_reg;
	uint32_t packet_count_hi_reg;
	uint32_t packet_count_lo_reg;
} axi_dump_gen;


typedef struct {
	uint64_t counter;
	axi_memory_writer_intr *writer_ptr;
} callback_ref;


int scugic_initialize(XScuGic *gic_ptr);
int scugic_connect(XScuGic *gic_ptr, uint32_t interrupt_id, void *handler, callback_ref *callback_ref_ptr, uint8_t priority);
void memwr_intr_handler(void *callback);


int main() {

	init_platform();

    XScuGic scugic;

    callback_ref ref;

    ref.writer_ptr = (axi_memory_writer_intr*)0x40000000;
    ref.counter = 0;

	axi_dump_gen *gen_ptr = (axi_dump_gen*)0x40010000;


    int status = scugic_initialize(&scugic);
    if (status != XST_SUCCESS){
		xil_printf("[APP] : execution stopped with error %d\r\n", status);
		return status;
	}

    status = scugic_connect(&scugic, MEMWR_INTR, memwr_intr_handler, &ref , 0x00);
    if (status != XST_SUCCESS){
        xil_printf("[APP] : execution stopped with error %d\r\n", status);
        return status;
    }

    Xil_ExceptionInit();
    Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT, (Xil_ExceptionHandler)XScuGic_InterruptHandler, &scugic);
    Xil_ExceptionEnableMask(XIL_EXCEPTION_ALL);
    Xil_ExceptionEnable();


    ////
    gen_ptr->packet_size_reg = 1024;
    gen_ptr->packet_limit_reg = 0x00000000;
    gen_ptr->pause_reg = 0x00000400;

    ref.writer_ptr->portion_size_reg = 1048576;
    ref.writer_ptr->memory_baseaddr_reg = 0x10000000;
    ref.writer_ptr->memory_highaddr_reg = 0x18000000;
    ref.writer_ptr->ctrl_reg = 0x06000003;

	gen_ptr->ctrl_reg = 0x00000001;

	volatile int reload_gen = 0;

	while(1){

	}

    cleanup_platform();
    return 0;
}




int     scugic_initialize(XScuGic *gic_ptr){
    int status = 0;

    XScuGic_Config *cfg;

    cfg = XScuGic_LookupConfig(XPAR_SCUGIC_0_DEVICE_ID);
    status = XScuGic_CfgInitialize(gic_ptr, cfg, cfg->CpuBaseAddress);
    if (status != XST_SUCCESS){
        return status;
    }

    return status ;
}




int scugic_connect(XScuGic *gic_ptr, uint32_t interrupt_id, void *handler, callback_ref *callback_ref_ptr, uint8_t priority){
    XScuGic_SetPriorityTriggerType(gic_ptr, interrupt_id, priority, 0x3);
    int status = XScuGic_Connect(gic_ptr, interrupt_id, (Xil_InterruptHandler)handler, callback_ref_ptr);

    if (status != XST_SUCCESS){
        return status;
    }

    XScuGic_Enable(gic_ptr, interrupt_id);
    return status;
}




void memwr_intr_handler(void *callback){

	callback_ref *callback_ref_ptr = (callback_ref*)callback;

	uint32_t fifo_current_volume = callback_ref_ptr->writer_ptr->fifo_current_volume;
	uint32_t current_address = callback_ref_ptr->writer_ptr->current_address_reg;
	uint32_t transferred_size = callback_ref_ptr->writer_ptr->transferred_size_reg;

//	int modulo = transferred_size % 8192;
//	if (modulo == 0){
//		printf("[INTR_%d] : 0x%08x : %d\r\n", ++intr_cnt, current_address, transferred_size);
//	} else {
//		printf("0x%08x : %d\r\n", current_address, transferred_size);
//	}

	Xil_DCacheInvalidateRange(current_address, transferred_size);

	uint64_t *data = (uint64_t*)current_address;

	for (int i = 0; i < transferred_size/8; i++){
		if (callback_ref_ptr->counter != data[i]){
			printf("0x%08x : %d\r\n", current_address, transferred_size);
			printf("Error : %d/%d\r\n", callback_ref_ptr->counter, data[i]);
			callback_ref_ptr->counter = data[i];
		}
		callback_ref_ptr->counter++;
	}



    return ;
}
