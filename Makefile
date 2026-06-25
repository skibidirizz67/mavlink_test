CC ?= gcc

SRCDIR := src
BLDDIR := build
BINDIR := bin
SCPDIR := user0@192.168.50.37:~/test
TARGET := $(BINDIR)/a

SRC := $(wildcard $(SRCDIR)/*.c)
OBJ := $(SRC:$(SRCDIR)/%.c=$(BLDDIR)/%.o)
DEP := $(OBJ:.o=.d)

CPPFLAGS :=
CFLAGS   := -O2 -Wall -Wextra -Wpedantic -Wno-address-of-packed-member -MMD -MP -I generated/include 
LDFLAGS  :=
LDLIBS   := -lpthread

all: $(TARGET)

$(TARGET): $(OBJ) | $(BINDIR)
	$(CC) $(LDFLAGS) $^ -o $@ $(LDLIBS)

$(BLDDIR)/%.o: $(SRCDIR)/%.c | $(BLDDIR)
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(BLDDIR) $(BINDIR):
	mkdir -p $@

scp: $(SRC)
	scp $< $(SCPDIR)/$<

-include $(DEP)

run: $(TARGET)
	./$(TARGET)

debug: CFLAGS := -g -O0 -Wall -Wextra -Wpedantic -MMD -MP
debug: $(TARGET)
	gdb --args ./$(TARGET)

clean:
	rm -rf $(BLDDIR) $(BINDIR)

.PHONY: scp all run debug clean