PROJECT = elsgui
BUILD_DIR = bin

CFILES = main.c

# TODO - you will need to edit these two lines!
DEVICE=stm32f103cbt6
OOCD_FILE = /home/pekka/stlink.cfg

# You shouldn't have to edit anything below here.


OPENCM3_DIR=../libopencm3

include $(OPENCM3_DIR)/mk/genlink-config.mk
include ../rules.mk
include $(OPENCM3_DIR)/mk/genlink-rules.mk
