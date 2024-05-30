#YOSYS          ?= yosys
#NEXTPNR        ?= nextpnr-himbaechel
#OPENFPGALOADER ?= openFPGALoader
YOSYS          ?= ../../yosys/yosys
NEXTPNR        ?= ../../nextpnr/nextpnr-himbaechel
OPENFPGALOADER ?= ../../openFPGALoader/build/openFPGALoader
GOWIN_PACK     ?= gowin_pack

BOARD   ?= tangnano20k
DEVICE  ?= GW2AR-LV18QN88C8/I7
FAMILY  ?= GW2A-18C
LEDS_NR ?= 6

PROJECTS  = pio
TOP      ?= top
OUTDIR    = out

pio_SRCS = ../src/top/${TOP}.v ../src/pio.v ../src/machine.v \
          ../src/pc.v ../src/decoder.v ../src/divider.v \
	  ../src/isr.v ../src/osr.v ../src/scratch.v ../src/fifo.v

all: $(patsubst %,$(OUTDIR)/%.fs,$(PROJECTS))

%: $(OUTDIR)/%.fs ;

sim-%:
	iverilog -DTESTBENCH -o $(OUTDIR)/$*.isim -s testbench -g2012 $*.sv
	$(OUTDIR)/$*.isim

flash-%: $(OUTDIR)/%.fs
	$(OPENFPGALOADER) -b $(BOARD) $<

$(OUTDIR)/%.fs: $(OUTDIR)/%.json.pnr
	$(GOWIN_PACK) -d $(FAMILY) -o $@ $<

$(OUTDIR)/%.json.pnr: $(OUTDIR)/%.json
	$(NEXTPNR) --json $< --write $@ --device $(DEVICE) --vopt family=$(FAMILY) --vopt cst=$(BOARD).cst

.SECONDEXPANSION:
$(OUTDIR)/%.json: $$(%_SRCS) | $(OUTDIR)
	$(YOSYS) -D LEDS_NR=$(LEDS_NR) -p "read_verilog -sv $^; synth_gowin -json $@"

$(OUTDIR):
	mkdir -p $(OUTDIR)

clean:
	$(RM) $(OUTDIR)/*

# Don't delete intermediate files
.PRECIOUS: $(OUTDIR)/%.json $(OUTDIR)/%.json.pnr

.PHONY: clean all
