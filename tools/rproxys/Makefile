OBJS = ./source/rproxys.o ./source/timer.o ./source/log.o

CFLAGS += -I./include -Wall

all: clean rproxys

rproxys: $(OBJS)
	@echo " [rproxys] CC -o $@"
	@$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^

PACK_DIR = proxy_server
PACK_FILES = rproxys rproxys.conf rproxys.service install.sh

clean:
	rm -f ./source/*.o rproxys

pack: rproxys
	@mkdir -p $(PACK_DIR)
	@cp -f $(PACK_FILES) $(PACK_DIR)/
	@tar -czf $(PACK_DIR).tar.gz $(PACK_DIR)
	@rm -rf $(PACK_DIR)
	@echo " [rproxys] Packed -> $(PACK_DIR).tar.gz"
