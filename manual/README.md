# T-Dongle-C5 user manual

Edit `manual.tex` and the images in `img/`. From the repository root, build with:

```sh
make -C manual
```

The Makefile runs XeLaTeX three times to resolve cross-references and the cover
overlay. The result is `manual/manual.pdf`; generated PDFs and LaTeX auxiliary
files are ignored by Git. The logo PDF in `img/` is a source asset and is tracked.

On Ubuntu, install the same dependencies used by CI:

```sh
sudo apt-get install --no-install-recommends \
  texlive-xetex texlive-latex-extra \
  texlive-fonts-recommended texlive-fonts-extra \
  fonts-crosextra-carlito inkscape
mkdir -p ~/.local/share/fonts
ln -s /usr/share/texlive/texmf-dist/fonts/opentype ~/.local/share/fonts/texlive-opentype
fc-cache -f
```

The fontconfig link makes TeX Live's Source Code Pro font available to XeLaTeX.
If the link already exists, skip the `ln` command.

CI builds the manual for pushes to `main`, pull requests, and manual workflow
runs. It uploads the `transmitter-manual` artifact and includes the PDF in
production Pages deployments at `manual.pdf`, linked from the installer.
