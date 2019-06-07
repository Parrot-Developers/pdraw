BIN := test-app
SRC := main.c
OBJ := $(SRC:.c=.o)

# Change this to point to your SDK dir
SDK_DIR := ../../../..
SDK_INC_DIR := $(SDK_DIR)/out/groundsdk-linux/sdk/usr/include
SDK_LIB_DIR := $(SDK_DIR)/out/groundsdk-linux/sdk/usr/lib
SDK_PROTOBUF_INC_DIR := $(SDK_INC_DIR)/libvideo-metadata-protobuf/generated

all: $(BIN)

$(BIN): $(OBJ)
	@gcc -o $@ -L$(SDK_LIB_DIR) -Wl,-rpath=$(SDK_LIB_DIR) $^ -lpdraw-backend

%.o: %.c
	@gcc -c -o $@ -I$(SDK_INC_DIR) -I$(SDK_PROTOBUF_INC_DIR) $<

clean:
	@rm -f $(OBJ) $(BIN)

.PHONY: clean
