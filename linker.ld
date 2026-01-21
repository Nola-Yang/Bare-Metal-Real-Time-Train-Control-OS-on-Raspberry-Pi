SECTIONS {
	. = 0x80000;            /* start text at this location */
	.text.boot : {          /* boot code must come first */
		KEEP(*(.text.boot))
	}
	.text : {               /* rest of code */
		KEEP(*(.text*))
	}

	. = 0x200000;           /* start data at this location for MMU setup */
	.rodata : {
		KEEP(*(.rodata*))
	}
	.data : {
		KEEP(*(.data*))
	}

	/* mark BSS segment */
	.bss : {
		KEEP(*(.bss*))
	}
	__bss_start = ADDR(.bss);
	__bss_end   = ADDR(.bss) + SIZEOF(.bss);

	/* Kernel stack (4KB, 16-byte aligned) */
	. = ALIGN(16);
	__kernel_stack_bottom = .;
	. = . + 0x1000;
	__kernel_stack_top = .;
}
