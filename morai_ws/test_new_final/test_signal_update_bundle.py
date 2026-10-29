"""Exercise the actual offline source updater with a temporary Git checkout."""
import importlib.util
import os
from pathlib import Path
import shutil
import stat
import subprocess
import tempfile
import unittest
import zipfile

REPO=Path(__file__).resolve().parents[2]
SCRIPTS=REPO/'morai_ws/docker/final_ws'


class SignalUpdateBundleTest(unittest.TestCase):
    def setUp(self):
        bash=Path('C:/Program Files/Git/bin/bash.exe')
        self.bash=str(bash) if bash.exists() else shutil.which('bash')
        if not self.bash or not shutil.which('git'):self.skipTest('Git/Bash unavailable')
        self.temp=tempfile.TemporaryDirectory();self.addCleanup(self.temp.cleanup)
        self.root=Path(self.temp.name)
        spec=importlib.util.spec_from_file_location('export_signal_bundle',SCRIPTS/'export_signal_update.py')
        module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
        archive=module.export_bundle(REPO,self.root/'update.zip')
        self.bundle=self.root/'bundle'
        with zipfile.ZipFile(archive) as z:
            paths=[n for n in z.namelist() if n.startswith('morai_ws/')]
            self.assertEqual(len(paths),10)
            self.archive_modes={i.filename:(i.create_system,i.external_attr >> 16) for i in z.infolist()}
            z.extractall(self.bundle)
        self.files=(self.bundle/'signal_update_files.txt').read_text().splitlines()
        self.target=self.root/'target';self.target.mkdir()
        subprocess.run(['git','-C',str(self.target),'init','-q'],check=True,capture_output=True)
        subprocess.run(['git','-C',str(self.target),'symbolic-ref','HEAD','refs/heads/test/roundabout'],check=True)
        self.old={}
        for name in self.files:
            p=self.target/name;p.parent.mkdir(parents=True,exist_ok=True)
            self.old[name]=('old '+name+'\n').encode();p.write_bytes(self.old[name])
        self.env=self.target/'morai_ws/docker/final_ws/highway.env'
        self.env.parent.mkdir(parents=True);self.env.write_bytes(b'COMPETITION_STATUS_PORT=9099\n')

    def apply(self):
        return subprocess.run([self.bash,str(self.bundle/'apply_signal_update.sh'),str(self.target)],capture_output=True)

    def test_updates_exactly_ten_sources_backs_up_and_preserves_env(self):
        result=self.apply();self.assertEqual(result.returncode,0,result.stderr)
        backups=list((self.target/'morai_ws/test-results').glob('signal-backup.*'))
        self.assertEqual(len(backups),1)
        for name in self.files:
            self.assertEqual((self.target/name).read_bytes(),(self.bundle/name).read_bytes())
            self.assertEqual((backups[0]/name).read_bytes(),self.old[name])
        self.assertEqual(self.env.read_bytes(),b'COMPETITION_STATUS_PORT=9099\n')

    def test_other_branch_is_rejected_before_any_overwrite(self):
        subprocess.run(['git','-C',str(self.target),'symbolic-ref','HEAD','refs/heads/final_ws'],check=True)
        result=self.apply();self.assertNotEqual(result.returncode,0)
        for name in self.files:self.assertEqual((self.target/name).read_bytes(),self.old[name])

    def test_corrupt_bundle_is_rejected_before_any_overwrite(self):
        (self.bundle/self.files[0]).write_bytes(b'corrupt payload')
        result=self.apply();self.assertNotEqual(result.returncode,0)
        for name in self.files:self.assertEqual((self.target/name).read_bytes(),self.old[name])

    def test_archive_permissions_are_readable_for_sudo_docker_readback(self):
        for name,(system,mode) in self.archive_modes.items():
            self.assertEqual(system,3,name)
            self.assertTrue(stat.S_ISREG(mode),name)
            executable=name.endswith('.sh') or ('/scripts/' in name and name.endswith('.py'))
            self.assertEqual(stat.S_IMODE(mode),0o755 if executable else 0o644,name)

    def test_permission_repair_preserves_contents_and_env(self):
        result=subprocess.run([self.bash,str(self.bundle/'repair_signal_permissions.sh'),str(self.target)],capture_output=True)
        self.assertEqual(result.returncode,0,result.stderr)
        for name in self.files:self.assertEqual((self.target/name).read_bytes(),self.old[name])
        self.assertEqual(self.env.read_bytes(),b'COMPETITION_STATUS_PORT=9099\n')
