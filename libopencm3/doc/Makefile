# Makefile to build libopencm3 documentation

# 14 September 2012
# (C) Ken Sarkies <ksarkies@internode.on.net>
# (C) Benjamin Vernoux <bvernoux@gmail.com>

doc: html latex

html: cm3 lpc43 top

cm3:
	cd cm3/; doxygen

lpc43:
	cd lpc43xx/; doxygen

top:
	doxygen

latex: lpc43.pdf

lpc43.pdf:
	cd lpc43xx/; doxygen Doxyfile_latex; cd latex/; $(MAKE); cp refman.pdf ../../lpc43.pdf

clean:
	@rm -rf html/ */html/ */latex/ *.pdf */*.tag

.PHONY: doc html cm3 lpc43 top latex

