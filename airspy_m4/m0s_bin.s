
	.section .rodata

	.global	m0s_bin
	.align  4
m0s_bin:
	.incbin "../airspy_m0s/airspy_m0s.bin"

	.global m0s_bin_size
	.align  4
m0s_bin_size:
	.int	m0s_bin_size - m0s_bin
