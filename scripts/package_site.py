#!/usr/bin/env python3
"""Package the web installer using ESP-IDF's actual flash offsets and images."""
import argparse
import hashlib
import json
from datetime import datetime, timezone
from pathlib import Path
import shutil


def package(build: Path, output: Path, version: str):
    build = build.resolve()
    args = json.loads((build / 'flasher_args.json').read_text())
    if args['extra_esptool_args']['chip'] != 'esp32c5':
        raise ValueError('Expected an ESP32-C5 build')
    if args['flash_settings']['flash_size'] != '16MB':
        raise ValueError('Expected a 16 MB flash configuration')
    files = args['flash_files']
    if len(files) != 3:
        raise ValueError('Expected bootloader, partition table and application')
    parts = []
    images = []
    end = 0
    names = set()
    for address, filename in sorted(files.items(), key=lambda item: int(item[0], 0)):
        offset = int(address, 0)
        path = (build / filename).resolve()
        if not path.is_relative_to(build):
            raise ValueError('Firmware image must be inside the build directory')
        data = path.read_bytes()
        if path.name in names or not data or offset < end or offset % 4096 or offset + len(data) > 16 * 1024 * 1024:
            raise ValueError('Duplicate, empty, overlapping or out-of-flash image')
        names.add(path.name)
        end = ((offset + len(data) + 4095) // 4096) * 4096
        parts.append(dict(name=path.name, offset=offset, size=len(data),
                          sha256=hashlib.sha256(data).hexdigest(), md5=hashlib.md5(data).hexdigest()))
        images.append(path)
    # Validate everything before creating the deployment artifact.
    if output.exists():
        raise ValueError(f'Output already exists: {output}; choose a fresh directory')
    shutil.copytree(Path(__file__).resolve().parents[1] / 'web', output)
    firmware = output / 'firmware'
    firmware.mkdir()
    for path in images:
        shutil.copy2(path, firmware / path.name)
    manifest = dict(schema_version=1, chip='ESP32-C5', flash_size='16MB', version=version,
                    built_at=datetime.now(timezone.utc).strftime('%Y-%m-%d %H:%M UTC'), parts=parts)
    (firmware / 'manifest.json').write_text(json.dumps(manifest, indent=2) + '\n')
    (output / '.nojekyll').touch()
    return manifest


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build', type=Path, default=Path('build'))
    parser.add_argument('--output', type=Path, default=Path('dist'))
    parser.add_argument('--version', required=True, help='Firmware source commit')
    options = parser.parse_args()
    package(options.build, options.output, options.version)
