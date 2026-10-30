#!/usr/bin/env python3
"""Write only LAN IPs and a fresh container name; preserve image and ports."""
import argparse
from datetime import datetime
from pathlib import Path
import re
import shutil
import uuid

PROFILES = {
    'personal': ('192.168.10.21', '192.168.10.14'),
    'competition': ('192.168.0.10', '192.168.0.1'),
}


def configure(directory, profile):
    ubuntu, morai = PROFILES[profile]
    target = directory / 'highway.env'
    if not target.exists():
        shutil.copyfile(directory / 'highway.env.example', target)
    content = target.read_text(encoding='utf-8')
    backup = target.with_name('highway.env.' + datetime.now().strftime('%Y%m%d-%H%M%S-%f') + '.bak')
    shutil.copyfile(target, backup)
    name = 'morai-real-final-' + profile + '-' + uuid.uuid4().hex[:8]
    for key, value in dict(UBUNTU_IP=ubuntu, MORAI_IP=morai, CONTAINER_NAME=name).items():
        if re.search(r'^' + key + r'=', content, re.M):
            content = re.sub(r'^' + key + r'=.*$', key + '=' + value, content, flags=re.M)
        else:
            content += '\n' + key + '=' + value + '\n'
    # pathlib.write_text(newline=...) is unavailable on Ubuntu's Python 3.8.
    with target.open('w', encoding='utf-8', newline='\n') as stream:
        stream.write(content)
    return dict(UBUNTU_IP=ubuntu, MORAI_IP=morai, CONTAINER_NAME=name), backup


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('profile', choices=PROFILES)
    args = parser.parse_args()
    settings, backup = configure(Path(__file__).resolve().parent, args.profile)
    print('\n'.join(k + '=' + v for k, v in settings.items()))
    print('Backup:', backup)
    print('Image, ports, sensor settings and OS NIC addresses were not changed.')
    print('Stop the previous launch/container before this change; run RUN_REAL_FINAL.sh install next.')
    if args.profile == 'competition':
        print('Confirm the organiser forwarding destination matches COMPETITION_STATUS_PORT in highway-test.env; Windows loopback 9099 is not assumed to be the Ubuntu port.')
