import {ESPLoader, Transport} from './vendor/esptool-js-0.7.0.js';
import {validateManifest, checkedImages} from './firmware.mjs';

const $ = id => document.getElementById(id);
const manifestUrl = new URL('./firmware/manifest.json', import.meta.url);
const supported = isSecureContext && 'serial' in navigator;
let manifest, loader, transport, busy = false;

// Version 0.7.0 inherits C6's SPI register base. C5 uses 0x60003000.
// Same correction as ESP-WebSDR; see Espressif's esptool ESP32-C5 target.
class C5Loader extends ESPLoader {
  async detectChip(...args) {
    await super.detectChip(...args);
    if (this.chip.CHIP_NAME === 'ESP32-C5') this.chip.SPI_REG_BASE = 0x60003000;
  }
}

function log(text) {
  $('log').textContent += text;
  $('log').scrollTop = $('log').scrollHeight;
}
function status(text, error = false) {
  $('status').textContent = text;
  $('status').dataset.error = String(error);
}
function buttons() {
  $('connect').disabled = !supported || !manifest || busy || !!transport;
  $('install').disabled = busy || !loader;
  $('disconnect').disabled = busy || !transport;
}
async function disconnect() {
  loader = null;
  const previous = transport;
  transport = null;
  $('device').textContent = 'Not connected';
  if (previous) await previous.disconnect();
}
async function cleanup() {
  try { await disconnect(); } catch (error) { log(`Port close: ${error.message}\n`); }
}

$('connect').onclick = async () => {
  if (busy || transport) return;
  busy = true;
  buttons();
  $('log').textContent = '';
  $('progress').value = 0;
  try {
    // Request from the click handler so the browser retains user activation.
    const port = await navigator.serial.requestPort();
    transport = new Transport(port, false);
    const candidate = new C5Loader({transport, baudrate: 115200,
      terminal: {clean() {}, write: log, writeLine: text => log(text + '\n')}});
    status('Connecting to the bootloader…');
    await candidate.main();
    if (candidate.chip.CHIP_NAME !== manifest.chip) {
      throw Error(`Detected ${candidate.chip.CHIP_NAME}. This firmware requires an ESP32-C5.`);
    }
    const size = await candidate.detectFlashSize();
    if (size !== manifest.flash_size) throw Error(`Detected ${size} flash; this firmware requires 16MB.`);
    loader = candidate;
    $('device').textContent = `ESP32-C5 · ${size} flash`;
    status('Connected. Select Install transmitter firmware to continue.');
  } catch (error) {
    status(error.name === 'NotFoundError' ? 'No device selected. Connect when ready.' :
      `${error.message} Close serial monitors and reconnect while holding the dongle’s button.`, true);
    await cleanup();
  } finally { busy = false; buttons(); }
};

$('install').onclick = async () => {
  if (busy || !loader) return;
  busy = true;
  buttons();
  $('progress').value = 0;
  try {
    status('Downloading and checking firmware…');
    const fileArray = await checkedImages(manifest, manifestUrl);
    const total = fileArray.reduce((sum, file) => sum + file.data.length, 0);
    status('Writing firmware. Keep the dongle connected and this page open.');
    await loader.writeFlash({fileArray, flashMode: 'keep', flashFreq: 'keep', flashSize: 'keep',
      eraseAll: false, compress: true,
      reportProgress(index, written, size) {
        const before = fileArray.slice(0, index).reduce((sum, file) => sum + file.data.length, 0);
        $('progress').value = 100 * (before + fileArray[index].data.length * written / size) / total;
      }});
    status('Verifying flash contents…');
    for (const part of manifest.parts) {
      const actual = await loader.flashMd5sum(part.offset, part.size);
      if (actual.toLowerCase() !== part.md5) throw Error(`Flash verification failed for ${part.name}.`);
      log(`Verified ${part.name}\n`);
    }
    $('progress').value = 100;
    status('Firmware installed and verified. Unplug and reconnect without holding the button to start transmitting.');
  } catch (error) {
    status(`Installation failed: ${error.message} Reconnect in BOOT mode and retry.`, true);
  } finally {
    await cleanup();
    busy = false;
    buttons();
  }
};

$('disconnect').onclick = async () => {
  if (busy) return;
  busy = true;
  buttons();
  await cleanup();
  status('Disconnected. Unplug and reconnect to start the installed firmware.');
  busy = false;
  buttons();
};
window.addEventListener('beforeunload', event => {
  if (busy) { event.preventDefault(); event.returnValue = ''; }
});
navigator.serial?.addEventListener('disconnect', () => {
  if (!busy && transport) {
    void cleanup().then(() => { status('Dongle disconnected. Reconnect to try again.'); buttons(); });
  }
});

async function initialize() {
  try {
    const response = await fetch(manifestUrl, {cache: 'no-store'});
    if (!response.ok) throw Error(`Firmware request failed (${response.status}).`);
    manifest = validateManifest(await response.json());
    $('version').textContent = `Build ${manifest.version.slice(0, 12)} · ${manifest.built_at}`;
    status(supported ? 'Ready. Connect your dongle to begin.' :
      'USB flashing requires a desktop browser that supports WebSerial over HTTPS (or localhost).', !supported);
  } catch (error) {
    manifest = null;
    $('version').textContent = 'Firmware unavailable';
    status(`Cannot load firmware: ${error.message} Reload the page to retry.`, true);
  }
  buttons();
}
buttons();
void initialize();
