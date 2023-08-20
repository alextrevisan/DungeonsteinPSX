include setup.mk

# Project target name
TARGET		= Dungeonstein

#EMULATOR
EMUBIN = D:\\Dev\\DevEngine3D\\pcsx-redux\\pcsx-redux.exe
EMU_CMD = $(EMUBIN) -iso iso/$(TARGET).cue
#EMU_CMD = $(NOPS) -exe bin/$(TARGET).exe com5 -fast
#EMU_CMD = $(MCOMMS) -dev com5 run bin/$(TARGET).exe

# Searches for C, C++ and S (assembler) files in local directory
CFILES		= $(notdir $(wildcard *.c))
CPPFILES 	= $(notdir $(wildcard *.cpp))
AFILES		= $(notdir $(wildcard *.s))

#UI
UI_DIR = ui
UI =  $(notdir $(wildcard $(UI_DIR)/*.s))
UIFILES = $(addprefix build/$(UI_DIR)/,$(UI:.s=.o)) \

#ENGINE
ENGINE_DIR = engine
ENGINE_CPP =  $(notdir $(wildcard $(ENGINE_DIR)/*.cpp))
ENGINE_C =  $(notdir $(wildcard $(ENGINE_DIR)/*.c))
ENGINEFILES = $(addprefix build/$(ENGINE_DIR)/,$(ENGINE_CPP:.cpp=.o) $(ENGINE_C:.c=.o))

#MAIN
OFILES		= $(addprefix build/,$(CFILES:.c=.o)) \
			$(addprefix build/,$(CPPFILES:.cpp=.o)) \
			$(addprefix build/,$(AFILES:.s=.o)) 


# Project specific include and library directories
# (use -I for include dirs, -L for library dirs)
INCLUDE	 	+=	-I. -Iscenario -I$(ENGINE_DIR) -I$(UI_DIR)
LIBDIRS		+=

# Libraries to link
LIBS		= -lpsxgpu_exe_nogprel -lpsxgte_exe_nogprel -lpsxspu_exe_nogprel -lpsxetc_exe_nogprel -lpsxapi_exe_nogprel -lc_exe_nogprel

# C compiler flags
CFLAGS		= -Wa,--strip-local-absolute -ffreestanding -fno-builtin -nostdlib -fdata-sections -ffunction-sections -fsigned-char -fno-strict-overflow -fdiagnostics-color=always -msoft-float -march=r3000 -mtune=r3000 -mabi=32 -mno-mt -mno-llsc -G8 -fno-pic -mno-abicalls -mgpopt -mno-extern-sdata -O3
# C++ compiler flags
CPPFLAGS	= $(CFLAGS) -fno-exceptions \
						-fno-rtti \
						-fno-unwind-tables \
						-fno-threadsafe-statics \
						-fno-use-cxa-atexit \
						-Wno-narrowing \
						-fconcepts-ts \
						-std=c++20
						
# Assembler flags
AFLAGS		= -msoft-float

# Linker flags
LDFLAGS		= -g -Ttext=0x80010000 -gc-sections \
                        -T $(PSN00B_BASE)/mipsel-none-elf/lib/ldscripts/elf32elmip.x

all: $(OFILES) $(UIFILES) $(ENGINEFILES)
	@mkdir -p $(BIN_FOLDER)
	$(LD) $(LDFLAGS) $(LIBDIRS) $(OFILES) $(UIFILES) $(ENGINEFILES) $(LIBS)  -o bin/$(TARGET)
	$(ELF2X) -q $(BIN_FOLDER)/$(TARGET) $(BIN_FOLDER)/$(TARGET).exe
	$(ELF2X) -q $(BIN_FOLDER)/$(TARGET) $(BIN_FOLDER)/$(TARGET).elf

iso: all
	@mkdir -p $(ISO_FOLDER)
	@rm -rf $(ISO_FOLDER)/*.cue $(ISO_FOLDER)/*.bin
	@$(MKPSXISO) $(MKPSXISO_XML)
	@mv *.cue $(ISO_FOLDER)
	@mv *.bin $(ISO_FOLDER)

run: iso
	$(EMU_CMD)

build/%.o: %.c
	@mkdir -p $(dir $@)
	$(CXX) $(CFLAGS) $(INCLUDE) -c $< -o $@
	
build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(AFLAGS) $(CPPFLAGS) $(INCLUDE) -c $< -o $@

build/%.o: %.s
	@mkdir -p $(dir $@)
	$(CC) $(AFLAGS) $(INCLUDE) -c $< -o $@

clean:
	rm -rf build $(BIN_FOLDER) $(ISO_FOLDER) *.bin *.cue