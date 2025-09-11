
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

INC_FOLDER+=$(PROJ_DIR)/../../robotun_lib/third_lib_2/include\

INC:=$(foreach inc, $(INC_FOLDER), -I$(inc))

.PHONY: all default clean 

LIBS+=$(PROJ_DIR)/../../robotun_lib/third_lib_2/lib/libmatplot.a

LIBS+=$(PROJ_DIR)/../../robotun_lib/third_lib_2/lib/Matplot++/libnodesoup.a


all: $(BUILD_DIR)/$(TARGET_NAME)
	@echo "Done."

$(OBJ_DIR)/%.opp: %.cpp
	@echo "Compiling $(notdir $<)"
	@mkdir -p $(shell dirname $@)
	@$(CPP) $(C_FLAG) $(INC) -c $< -o $@

$(BUILD_DIR)/$(TARGET_NAME): $(OBJ_FILES)
	@echo "Link $(notdir $@)"
	@mkdir -p $(shell dirname $@)
	@$(CPP) $(C_FLAG) $(INC) $(OBJ_FILES) $(LIBS) -o $@

clean:
	@echo "Remove build"
	@rm -r $(BUILD_DIR)

run: $(BUILD_DIR)/$(TARGET_NAME)
	@echo "$(notdir $<) begin : "
	@echo " "
	@$(BUILD_DIR)/$(TARGET_NAME).exe
	@echo " "
	@echo "end."
