# Browser installer

This buildless site is packaged with the firmware by `scripts/package_site.py`.
All asset and firmware URLs are relative, including when served at
`https://espargos.net/t-dongle-c5-transmitter/`.

The self-hosted esptool.js 0.7.0 bundle and its licenses were copied from
`ESPARGOS/esp-web-sdr`. Upstream release and SHA-256 checksums are recorded in
`vendor/esptool-js-provenance.json`. The application applies the same C5
`SPI_REG_BASE = 0x60003000` correction as that project's installer; recheck this
against Espressif's `esptool/targets/esp32c5.py` before upgrading the bundle.
Carlito is bundled in `fonts.css`, including its SIL Open Font License.

The installer requires a desktop browser that supports WebSerial, over HTTPS or localhost.
It checks the chip and 16 MB flash size, downloads and validates all images
before writing, then checks their MD5 against flash readback. The SHA-256 checks
also detect stale assets after a deployment. These hashes detect corruption;
they are not firmware signatures. No full-chip erase is performed, preserving
NVS settings with the current partition layout. Restarting after installation
is deliberately manual: unplug and reconnect without holding the button.

To preview a local build from the repository root:

```sh
rm -rf dist/
idf.py build
python3 scripts/package_site.py --version "$(git rev-parse HEAD)"
make -C manual
cp manual/manual.pdf dist/manual.pdf
python3 -m http.server 8000 --directory dist
```

Open `http://localhost:8000/`. The packager requires a new output directory;
remove the generated `dist/` before packaging again.
