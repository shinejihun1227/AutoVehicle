#!/usr/bin/env python3
"""Export the six runtime files for the cumulative test/roundabout signal fix."""
import argparse
import hashlib
from pathlib import Path
import subprocess
import zipfile


def export_bundle(repo, destination):
    scripts=repo/'morai_ws/docker/final_ws'
    files=(scripts/'signal_update_files.txt').read_text().splitlines()
    if len(files)!=6 or len(set(files))!=6:
        raise ValueError('Expected exactly six runtime files')
    payload={}
    for file in files:
        if not file.startswith('morai_ws/src/') or '..' in Path(file).parts:
            raise ValueError('Invalid update path: '+file)
        payload[file]=(repo/file).read_bytes().replace(b'\r\n',b'\n')
    payload['apply_signal_update.sh']=(scripts/'apply_signal_update.sh').read_bytes().replace(b'\r\n',b'\n')
    payload['signal_update_files.txt']=('\n'.join(files)+'\n').encode()
    payload['README_KO.md']=(repo/'morai_ws/docs/20261030_SIGNAL_TRANSFER_RIGHT_KO.md').read_bytes().replace(b'\r\n',b'\n')
    payload['REVISION.txt']=(subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip()+'\n').encode()
    payload['SHA256SUMS']=''.join(hashlib.sha256(data).hexdigest()+'  '+name+'\n'
                                for name,data in sorted(payload.items())).encode()
    destination.parent.mkdir(parents=True,exist_ok=True)
    with zipfile.ZipFile(destination,'w',zipfile.ZIP_DEFLATED) as archive:
        for name,data in payload.items():archive.writestr(name,data)
    return destination


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    args=parser.parse_args()
    print(export_bundle(Path(__file__).resolve().parents[3],args.output))
