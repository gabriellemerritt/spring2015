BINDIR=bin
OUT=${BINDIR}/dymxl.elf
TIVAWARE=../tivaware
DEFS=-DTARGET_IS_SNOWFLAKE_RA0
CFLAGS=${DEFS} -mthumb -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=softfp -ffunction-sections -fdata-sections -MD -std=c99 -Wall -pedantic -g
TC=arm-none-eabi-
OPENOCD_SCRIPTS=../openocd

default: ${BINDIR} ${OUT}.bin

${BINDIR}:
	mkdir -p ${BINDIR}

${OUT}: dymxl.c startup.c
	${TC}gcc ${CFLAGS} -I${TIVAWARE} dymxl.c startup.c -L${TIVAWARE}/driverlib/gcc -lc -lm -ldriver -T dymxl.ld -Wl,--gc-sections -Wl,--entry=ResetISR -o ${OUT}
	${TC}objdump -S -d ${OUT} > ${OUT}.objdump

${OUT}.bin: ${OUT}
	${TC}objcopy -O binary ${OUT} ${OUT}.bin

clean:
	rm ${BINDIR}/*
	rmdir ${BINDIR}

program: ${OUT}.bin
	sudo openocd -f /usr/local/share/openocd/scripts/interface/olimex-arm-usb-tiny-h.cfg -f ${OPENOCD_SCRIPTS}/tm4c129.cfg -c "init; halt;  flash write_image erase ${OUT}.bin; sleep 100; reset run; shutdown"

gdb_server: $(OUT).bin
	sudo openocd -f /usr/local/share/openocd/scripts/interface/olimex-arm-usb-tiny-h.cfg -f $(OPENOCD_SCRIPTS)/tm4c129.cfg -c "init; halt"

gdb: $(OUT).bin
	$(TC)gdb $(OUT) -x gdb_init_commands
