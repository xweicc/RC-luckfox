OBJS = ./source/rproxys.o ./source/timer.o

CFLAGS += -I./include -Wall

all: clean rproxys

rproxys: $(OBJS)
	@echo " [rproxys] CC -o $@"
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

clean:
	rm -f ./source/*.o rproxys
