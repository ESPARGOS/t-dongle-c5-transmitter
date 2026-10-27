MAIN = manual
LATEX = xelatex
LATEXFLAGS = -interaction=nonstopmode -halt-on-error

IMAGES = \
	img/espargos-logo.pdf \
	img/dongle-in-powerbank.png

.PHONY: all clean booklet

all: $(MAIN).pdf

$(MAIN).pdf: $(MAIN).tex $(IMAGES)
	# Three passes so cross-references and the TikZ cover overlay settle.
	$(LATEX) $(LATEXFLAGS) $(MAIN).tex
	$(LATEX) $(LATEXFLAGS) $(MAIN).tex
	$(LATEX) $(LATEXFLAGS) $(MAIN).tex

# Regenerate the ESPARGOS logo PDF from SVG (requires Inkscape)
img/espargos-logo.pdf: img/espargos-logo.svg
	inkscape --export-type=pdf --export-filename=$@ $<

# Print-ready A4 booklet (requires pdfbook2: pip install pdfbook2)
booklet: $(MAIN).pdf
	pdfbook2 --paper=a4paper $(MAIN).pdf

clean:
	rm -f $(MAIN).aux $(MAIN).log $(MAIN).out $(MAIN).toc \
	       $(MAIN).synctex.gz $(MAIN).fls $(MAIN).fdb_latexmk $(MAIN).xdv
