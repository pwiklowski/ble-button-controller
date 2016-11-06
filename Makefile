PROJECT = ble-beacon-button_controler

NRF51_SDK    = FILLME
GCC_LOCATION = FILLME2

LIB_DIRS     = -L$(GCC_LOCATION)/lib/gcc/arm-none-eabi/4.9.3/armv6-m

CC          := $(GCC_LOCATION)/bin/arm-none-eabi-gcc
CXX         := $(GCC_LOCATION)/bin/arm-none-eabi-g++
AS          := $(GCC_LOCATION)/bin/arm-none-eabi-gcc -x assembler-with-cpp
SIZE        := $(GCC_LOCATION)/bin/arm-none-eabi-size
OBJCOPY     := $(GCC_LOCATION)/bin/arm-none-eabi-objcopy

LD_SCRIPT_LOCATION := $(shell pwd)/setup/

LD_SCRIPT := gcc_nrf51_s110_xxaa.ld

BUILD_DIR = bin/
  
 
INC_DIRS +=-I"$(shell pwd)"
INC_DIRS +=-I"$(shell pwd)/setup"
INC_DIRS += -I"$(NRF51_SDK)/components/softdevice/s120/headers/"
INC_DIRS += -I"$(NRF51_SDK)/components/softdevice/common/softdevice_handler/"
INC_DIRS += -I"$(NRF51_SDK)/components/libraries/util/"
INC_DIRS += -I"$(NRF51_SDK)/components/device/"
INC_DIRS += -I"$(NRF51_SDK)/components/toolchain/gcc"
INC_DIRS += -I"$(NRF51_SDK)/components/toolchain"
INC_DIRS += -I"$(NRF51_SDK)/components/drivers_nrf/hal"
INC_DIRS += -I"$(NRF51_SDK)/components/ble/common"
INC_DIRS += -I"$(NRF51_SDK)/components/libraries/scheduler"
INC_DIRS += -I"$(NRF51_SDK)/components/libraries/gpiote"
INC_DIRS += -I"$(NRF51_SDK)/components/libraries/timer"
INC_DIRS += -I"$(NRF51_SDK)/components/libraries/button"
INC_DIRS += -I"$(NRF51_SDK)/examples/bsp/"
INC_DIRS += -I"$(NRF51_SDK)/external/rtx/include"
INC_DIRS += -I"$(NRF51_SDK)/components/ble/ble_services/ble_ans_c/"
INC_DIRS += -I"$(NRF51_SDK)/Include/serialization/connectivity/"
INC_DIRS += -I"$(NRF51_SDK)/components/ble/device_manager"
INC_DIRS += -I"$(NRF51_SDK)/components/drivers_nrf/pstorage"
INC_DIRS += -I"$(NRF51_SDK)/components/libraries/trace"
INC_DIRS += -I$(NRF51_SDK)/components/drivers_nrf/simple_uart
INC_DIRS += -I$(NRF51_SDK)/components/ble/device_manager/
INC_DIRS += -I$(NRF51_SDK)/components/ble/ble_db_discovery/

SRCS_DIRS += $(shell pwd)/setup
SRCS_DIRS += $(shell pwd)
SRCS_DIRS += $(NRF51_SDK)/components/libraries/button/
SRCS_DIRS += $(NRF51_SDK)/components/softdevice/s110/headers/
SRCS_DIRS += $(NRF51_SDK)/components/softdevice/common/softdevice_handler/
SRCS_DIRS += $(NRF51_SDK)/components/libraries/util/
SRCS_DIRS += $(NRF51_SDK)/components/device/
SRCS_DIRS += $(NRF51_SDK)/components/toolchain
SRCS_DIRS += $(NRF51_SDK)/components/drivers_nrf/hal
SRCS_DIRS += $(NRF51_SDK)/components/ble/common
SRCS_DIRS += $(NRF51_SDK)/components/libraries/scheduler
SRCS_DIRS += $(NRF51_SDK)/components/libraries/gpiote
SRCS_DIRS += $(NRF51_SDK)/components/libraries/timer
SRCS_DIRS += $(NRF51_SDK)/components/libraries/button
SRCS_DIRS += $(NRF51_SDK)/examples/bsp/
SRCS_DIRS += $(NRF51_SDK)/external/rtx/include
SRCS_DIRS += $(NRF51_SDK)/components/ble/ble_services/ble_ans_c/
SRCS_DIRS += $(NRF51_SDK)/components/libraries/trace/
SRCS_DIRS += $(NRF51_SDK)/components/drivers_nrf/pstorage/
SRCS_DIRS += $(NRF51_SDK)/components/ble/device_manager/
SRCS_DIRS += $(NRF51_SDK)/components/drivers_nrf/simple_uart/
SRCS_DIRS += $(NRF51_SDK)/components/ble/device_manager/
SRCS_DIRS += $(NRF51_SDK)/components/ble/ble_db_discovery/

C_SRCS += $(NRF51_SDK)/components/softdevice/common/softdevice_handler/softdevice_handler.c
C_SRCS += $(NRF51_SDK)/components/libraries/button/app_button.c 
C_SRCS += $(NRF51_SDK)/components/libraries/util/app_error.c 
C_SRCS += $(NRF51_SDK)/components/libraries/gpiote/app_gpiote.c 
C_SRCS += $(NRF51_SDK)/components/libraries/scheduler/app_scheduler.c 
C_SRCS += $(NRF51_SDK)/components/libraries/util/app_util_platform.c 
C_SRCS += $(NRF51_SDK)/components/libraries/timer/app_timer.c 
C_SRCS += $(NRF51_SDK)/components/libraries/util/nrf_assert.c 
C_SRCS += $(NRF51_SDK)/components/drivers_nrf/hal/nrf_delay.c 
C_SRCS += $(NRF51_SDK)/components/drivers_nrf/pstorage/pstorage.c 
C_SRCS += $(NRF51_SDK)/components/ble/common/ble_advdata.c 
C_SRCS += $(NRF51_SDK)/components/ble/common/ble_conn_params.c
C_SRCS += $(NRF51_SDK)/components/ble/common/ble_srv_common.c
C_SRCS += $(NRF51_SDK)/components/toolchain/system_nrf51.c
C_SRCS += $(NRF51_SDK)/examples/bsp/bsp.c
C_SRCS += $(NRF51_SDK)/components/libraries/trace/app_trace.c 
C_SRCS += $(NRF51_SDK)/components/ble/device_manager/device_manager_central.c 
C_SRCS += $(NRF51_SDK)/components/drivers_nrf/simple_uart/simple_uart.c
C_SRCS += $(NRF51_SDK)/components/ble/ble_db_discovery/ble_db_discovery.c 

C_SRCS += $(shell pwd)/main.c


CXX_EXT = cpp
C_EXT = c
AS_EXT = S

ELF = $(BUILD_DIR)$(PROJECT).elf
HEX = $(BUILD_DIR)$(PROJECT).hex
BIN = $(BUILD_DIR)$(PROJECT).bin
LSS = $(BUILD_DIR)$(PROJECT).lss
DMP = $(BUILD_DIR)$(PROJECT).dmp

#CXX_SRCS = $(wildcard $(patsubst %, %/*.$(CXX_EXT), . $(SRCS_DIRS)))
#C_SRCS += $(wildcard $(patsubst %, %/*.$(C_EXT), . $(SRCS_DIRS)))
AS_SRCS = $(wildcard $(patsubst %, %/*.$(AS_EXT), . $(SRCS_DIRS)))

VPATH = $(SRCS_DIRS)



CXX_OBJS = $(addprefix $(BUILD_DIR), $(notdir $(CXX_SRCS:.$(CXX_EXT)=.o)))
C_OBJS = $(addprefix $(BUILD_DIR), $(notdir $(C_SRCS:.$(C_EXT)=.o)))
AS_OBJS = $(addprefix $(BUILD_DIR), $(notdir $(AS_SRCS:.$(AS_EXT)=.o)))



# core flags
CORE_FLAGS = -mcpu=cortex-m0 -mthumb -mabi=aapcs

BLE_FLAGS =  -DBLE_STACK_SUPPORT_REQD -DBOARD_PCA10001 -DNRF51 -DDSOFTDEVICE_PRESENT -DNRF51822_QFAA_CA -DS110 -DDEBUG_NRF_USER 

CXX_FLAGS =  $(BLE_FLAGS) -g -ggdb3 -fno-rtti -fno-exceptions -fverbose-asm -Wa,-ahlms=$(BUILD_DIR)$(notdir $(<:.$(CXX_EXT)=.lst))
C_FLAGS = -std=gnu11 $(BLE_FLAGS) -g -ggdb3 -fverbose-asm -Wa,-ahlms=$(BUILD_DIR)$(notdir $(<:.$(C_EXT)=.lst))
AS_FLAGS = $(BLE_FLAGS) -g -ggdb3 -Wa,-amhls=$(BUILD_DIR)$(notdir $(<:.$(AS_EXT)=.lst))
LD_FLAGS = -T$(LD_SCRIPT_LOCATION)$(LD_SCRIPT) -g -Wl,-Map=$(BUILD_DIR)$(PROJECT).map,--cref,--no-warn-mismatch --specs=nano.specs -lc -lnosys -Wl,--gc-sections

#optimizze
LD_FLAGS += -Wl,--gc-sections
OPTIMIZATION += -ffunction-sections -fdata-sections -fno-strict-aliasing -flto -fno-builtin


CXX_FLAGS_F =  $(CORE_FLAGS) $(OPTIMIZATION) $(CXX_FLAGS)  $(CXX_DEFS) -MD -MP -MF $(BUILD_DIR)$(@F:.o=.d) $(INC_DIRS)
C_FLAGS_F =  $(CORE_FLAGS) $(OPTIMIZATION) $(C_FLAGS) $(C_DEFS) -MD -MP -MF $(BUILD_DIR)$(@F:.o=.d) $(INC_DIRS)
AS_FLAGS_F = $(CORE_FLAGS) $(AS_FLAGS) $(AS_DEFS) -MD -MP -MF $(BUILD_DIR)$(@F:.o=.d) $(INC_DIRS)
LD_FLAGS_F = $(CORE_FLAGS) $(LD_FLAGS) $(LIB_DIRS_F)


OBJS = $(AS_OBJS) $(C_OBJS) $(CXX_OBJS)
DEPS = $(OBJS:.o=.d)
all : mkdir $(ELF)  $(HEX) $(BIN) print_size


$(OBJS) : Makefile
$(ELF) : $(LD_SCRIPT)


$(ELF) : $(OBJS)
	$(CC) $(LD_FLAGS_F) $(OBJS) $(LIBS) -o $@

$(BUILD_DIR)%.o : %.$(AS_EXT)
	$(AS) -c $(AS_FLAGS_F) $< -o $@

$(BUILD_DIR)%.o : %.$(CXX_EXT)
	$(CXX) -c $(CXX_FLAGS_F) $< -o $@

$(BUILD_DIR)%.o : %.$(C_EXT)
	$(CC) -c $(C_FLAGS_F) $< -o $@

$(HEX) : $(ELF)
	$(OBJCOPY) -O ihex $< $@

$(BIN) : $(ELF)
	$(OBJCOPY) -O binary $< $@

print_size :
	$(SIZE) -B  -t --common $(OBJS) $(USER_OBJS)
	$(SIZE) -B  $(ELF)

mkdir :
	mkdir bin -p
clean:
	rm -r bin


flash:
