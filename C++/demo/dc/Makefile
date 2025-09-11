
TARGET_NAME=dc
BUILD_DIR=_build
OBJ_DIR=$(BUILD_DIR)/obj
PROJ_DIR=./

# Toolchain
CPP=g++


#FLAG
C_FLAG=-Wall -std=c++17

SRC_EXC_FILES=\

SRC_INC_FILES+=$(wildcard *.cpp)

SRC_FILES:=$(filter-out $(SRC_EXC_FILES), $(SRC_INC_FILES))

OBJ_FILES:=$(foreach obj, $(SRC_FILES), $(OBJ_DIR)/$(obj:%.cpp=%.opp))

INC_FOLDER+=$(PROJ_DIR)/../../robotun_lib/third_lib/include\

INC:=$(foreach inc, $(INC_FOLDER), -I$(inc))

.PHONY: all default clean 

# LIBS+=$(PROJ_DIR)/../../robotun_lib/third_lib/lib/matplot.lib
# LIBS+=$(PROJ_DIR)/../../robotun_lib/third_lib/lib/Matplot++/nodesoup.lib


LIB_DIR=-L$(PROJ_DIR)/../../robotun_lib/third_lib/lib/\
	-L$(PROJ_DIR)/../../robotun_lib/third_lib/lib/Matplot++/\


LIBS=-lnodesoup -lmatplot
LIBS+=-lgdi32
LIBS+=-lkernel32
LIBS+=-luser32
LIBS+=-lwinspool
LIBS+=-lshell32
LIBS+=-lole32
LIBS+=-loleaut32
LIBS+=-luuid
LIBS+=-lcomdlg32
LIBS+=-ladvapi32

# Linker flags. Use -L for the directory and -l for the library name.
LDFLAGS = $(LIB_DIR) $(LIBS)

all: $(BUILD_DIR)/$(TARGET_NAME)
	@echo "Done."

$(OBJ_DIR)/%.opp: %.cpp
	@echo "Compiling $(notdir $<)"
	@mkdir -p $(shell dirname $@)
	@$(CPP) $(C_FLAG) $(INC) -c $< -o $@

$(BUILD_DIR)/$(TARGET_NAME): $(OBJ_FILES)
	@echo "Link $(notdir $@)"
	@mkdir -p $(shell dirname $@)
	@$(CPP) $(C_FLAG) $(INC) $(OBJ_FILES) $(LDFLAGS) -o $@

clean:
	@echo "Remove build"
	@rm -r $(BUILD_DIR)

run: $(BUILD_DIR)/$(TARGET_NAME)
	@echo "$(notdir $<) begin : "
	@echo " "
	@$(BUILD_DIR)/$(TARGET_NAME).exe
	@echo " "
	@echo "end."
