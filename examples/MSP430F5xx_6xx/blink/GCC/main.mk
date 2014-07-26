DEVICE      = MSP430F5529
CC          = msp430-gcc
OBJCOPY     = msp430-objcopy
AR          = msp430-ar
OS 			= $(shell uname -s)
######################################
INCDIR      = ../../../../driverlib/MSP430F5xx_6xx
######################################
ifeq ($(OS),Cygwin)
GCCINCDIR   = $(shell cygpath -m $(dir $(shell which $(CC)))../../../../ccs_base/msp430/include_gcc)
else ifeq ($(OS),Darwin)
GCCINCDIR   = /opt/local/msp430
else ifeq ($(OS),Linux)
GCCINCDIR   = /usr/local/msp430
endif
LDDIR       = $(GCCINCDIR)/lib/ldscripts
CFLAGS      = -O2 -D__$(DEVICE)__ -mmcu=$(DEVICE) -ffunction-sections -fdata-sections
LDFLAGS     = -T $(LDDIR)/msp430.x -mmcu=$(DEVICE) -Wl,--gc-sections
LDFLAGS    += -L$(LDDIR)/$(shell echo $(DEVICE) | tr A-Z a-z)
######################################
EXECUTABLE  = main
HEXFILE     = main.hex
EXSRCDIR    = ..
SRCDIR      = $(INCDIR)
OBJDIR      = objs
EXOBJECT    = $(OBJDIR)/$(EXECUTABLE).o
SOURCES     = $(wildcard $(SRCDIR)/*.c)
OBJECTS     = $(patsubst %.c,$(OBJDIR)/%.o,$(notdir $(SOURCES)))
######################################
INCLUDES    = -I$(GCCINCDIR) -I$(INCDIR) -DDRIVERLIB_LEGACY_MODE
######################################

all: init $(HEXFILE)

init:
	mkdir -p $(OBJDIR)

$(HEXFILE): $(EXECUTABLE)
	$(OBJCOPY) -O ihex $(EXECUTABLE) $(HEXFILE)

$(EXECUTABLE): $(EXOBJECT) $(OBJECTS)
	$(CC) $(LDFLAGS) $(EXOBJECT) $(OBJECTS) -o $@

$(EXOBJECT): $(EXSRCDIR)/$(EXECUTABLE).c
	$(CC) $(INCLUDES) $(CFLAGS) -c $< -o $@

$(OBJECTS): $(OBJDIR)/%.o: $(SRCDIR)/%.c
	$(CC) $(INCLUDES) $(CFLAGS) -c $< -o $@

erase:
	mspdebug rf2500 "erase"
 
flash:
	mspdebug rf2500 "prog main"

test:
	mspdebug rf2500 "prog test.txt"
	
clean:
	rm -rf $(OBJDIR)
	rm -rf $(EXECUTABLE)
	rm -rf $(HEXFILE)
