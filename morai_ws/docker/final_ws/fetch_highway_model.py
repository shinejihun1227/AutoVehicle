#!/usr/bin/env python3
"""Fetch the exact team checkpoint and verify it before any torch.load."""
import hashlib
from pathlib import Path
import urllib.request

REVISION = '13b740689c280d04fd456b00b854efbd793c38c2'
SHA256 = '0784b486d445480640df15893bf499a0e99aa05a58e859f03fa80393435a5b3b'
URL = ('https://raw.githubusercontent.com/hyunho0429/ROI/' + REVISION +
       '/src/detection/camera_perception/models/best.pt')


def fetch(destination):
    destination=Path(destination)
    if destination.exists() and hashlib.sha256(destination.read_bytes()).hexdigest()==SHA256:
        print('HIGHWAY_MODEL_OK', destination)
        return
    destination.parent.mkdir(parents=True,exist_ok=True)
    temporary=destination.with_suffix('.download')
    digest=hashlib.sha256()
    try:
        with urllib.request.urlopen(URL,timeout=60) as source, temporary.open('wb') as output:
            while True:
                chunk=source.read(1024*1024)
                if not chunk: break
                digest.update(chunk); output.write(chunk)
        if digest.hexdigest()!=SHA256: raise ValueError('Highway model SHA256 mismatch')
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)
    print('HIGHWAY_MODEL_OK',destination)


if __name__=='__main__':
    fetch(Path(__file__).resolve().parents[2]/'src/detection/camera_perception/models/highway_best.pt')
