export function validateManifest(manifest) {
  if (manifest?.schema_version !== 1 || manifest.chip !== 'ESP32-C5' || manifest.flash_size !== '16MB' ||
      typeof manifest.version !== 'string' || !Array.isArray(manifest.parts) || manifest.parts.length !== 3) {
    throw Error('Invalid transmitter firmware manifest.');
  }
  let end = 0;
  const names = new Set();
  for (const part of manifest.parts) {
    if (!/^[\w-]+\.bin$/.test(part.name) || names.has(part.name) ||
        !Number.isSafeInteger(part.offset) || part.offset < end || part.offset % 4096 !== 0 ||
        !Number.isSafeInteger(part.size) || part.size <= 0 || part.offset + part.size > 16 * 1024 * 1024 ||
        !/^[a-f0-9]{64}$/.test(part.sha256) || !/^[a-f0-9]{32}$/.test(part.md5)) {
      throw Error('Invalid firmware image layout or checksum.');
    }
    names.add(part.name);
    end = Math.ceil((part.offset + part.size) / 4096) * 4096;
  }
  return manifest;
}

export async function checkedImages(manifest, manifestUrl) {
  validateManifest(manifest);
  const files = [];
  for (const part of manifest.parts) {
    const url = new URL(part.name, manifestUrl);
    url.searchParams.set('sha256', part.sha256);
    const response = await fetch(url, {cache: 'no-store'});
    if (!response.ok) throw Error(`Cannot download ${part.name} (${response.status}).`);
    const data = new Uint8Array(await response.arrayBuffer());
    const digest = Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', data)),
      byte => byte.toString(16).padStart(2, '0')).join('');
    if (data.length !== part.size || digest !== part.sha256) {
      throw Error(`Integrity check failed for ${part.name}. Reload this page and retry.`);
    }
    files.push({address: part.offset, data});
  }
  return files;
}
