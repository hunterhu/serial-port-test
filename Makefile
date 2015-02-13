EXEC := test_serial_port

objects := test_serial_port.o

CFLAGS := -g
CFLAGS += -Wall 
CFLAGS += -std=c99
CFLAGS += -I./include
CFLAGS += -D_POSIX_C_SOURCE=199309L
CFLAGS += -D_BSD_SOURCE

CC := arm-linux-gnueabihf-gcc

vpath %.c src

$(EXEC) : $(objects)
	$(CC) -o $@ $^ $(CFLAGS)

.PHONY : clean
clean :
	rm -f $(EXEC) $(objects)
