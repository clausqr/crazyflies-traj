"""
The rr harness guards, witnessed on the host without Docker: `run.bash --print`
shows the docker command it would run, and every refusal exits before Docker is
ever called (a refused run cannot start a container).
"""

import os
import subprocess

import pytest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def sh(script, *args, **env):
    e = {k: v for k, v in os.environ.items() if not k.startswith(('RR_', 'ROS_', 'FASTRTPS'))}
    e.update(env)
    return subprocess.run(['bash', f'scripts/{script}', *args], cwd=ROOT, env=e,
                          capture_output=True, text=True, timeout=30)


@pytest.fixture
def dds(tmp_path):
    """A profile folder (RR_DDS_DIR) holding the files run.bash selects."""
    for f in ('fastdds_loopback.xml', 'cyclonedds_loopback.xml', 'cyclonedds_client.xml'):
        (tmp_path / f).write_text('<profiles/>')
    (tmp_path / 'fastdds_client.xml').write_text('<profiles><address>192.0.2.7</address></profiles>')
    return tmp_path


def test_default_command_is_lean_and_contained(dds):
    r = sh('run.bash', '--print', RR_DDS_DIR=str(dds))
    assert r.returncode == 0, r.stderr
    cmd = r.stdout
    assert 'ROS_DOMAIN_ID=66' in cmd and 'RR_NET_MODE=lan' in cmd
    assert 'FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/config/fastdds_client.xml' in cmd
    assert f'{dds}:/ros2_ws/config:ro' in cmd
    assert 'motd.sh' in cmd                       # the interactive shell prints the MOTD
    assert '/traj:ro' in cmd
    for forbidden in ('/dev/bus/usb', '--device', '--privileged', '--gpus', 'plugdev',
                      '189:'):
        assert forbidden not in cmd, forbidden


def test_usb_refused(dds):
    r = sh('run.bash', '--print', RR_DDS_DIR=str(dds), RR_USB_ENABLED='1')
    assert r.returncode == 3 and 'never touches USB' in r.stderr
    assert r.stdout == ''


@pytest.mark.parametrize('dom', ['233', '1000', 'abc', ''])
def test_bad_domain_refused(dds, dom):
    r = sh('run.bash', '--print', RR_DDS_DIR=str(dds), RR_ROS_DOMAIN_ID=dom)
    if dom == '':
        # empty -> setup.env default 66 (":-66"), which is valid for a lab run
        assert r.returncode == 0 and 'ROS_DOMAIN_ID=66' in r.stdout
    else:
        assert r.returncode == 3, (dom, r.stdout)


def test_bad_net_mode_refused(dds):
    r = sh('run.bash', '--print', RR_DDS_DIR=str(dds), RR_NET_MODE='wifi')
    assert r.returncode == 3


def test_missing_profile_refused_with_the_download_command(tmp_path):
    for mode in ('loopback', 'lan'):
        r = sh('run.bash', '--print', RR_DDS_DIR=str(tmp_path / 'nope'), RR_NET_MODE=mode)
        assert r.returncode == 3 and r.stdout == ''
        assert 'curl -fu USER:PASSWORD -o' in r.stderr and '/dds/' in r.stderr


def test_lan_profile_with_own_ip_marker_refused_real_ip_passes(dds):
    """Seeded violation: the downloaded profile still holds __OWN_IP__ -> refused
    before Docker with the sed hint; the same file with a real IP runs."""
    prof = dds / 'fastdds_client.xml'
    prof.write_text('<profiles><address>__OWN_IP__</address></profiles>')
    r = sh('run.bash', '--print', RR_DDS_DIR=str(dds), RR_NET_MODE='lan')
    assert r.returncode == 3 and r.stdout == ''
    assert '__OWN_IP__' in r.stderr and 'sed -i' in r.stderr
    prof.write_text('<profiles><address>192.0.2.7</address></profiles>')
    r = sh('run.bash', '--print', RR_DDS_DIR=str(dds), RR_NET_MODE='lan')
    assert r.returncode == 0, r.stderr
    assert f'{dds}:/ros2_ws/config:ro' in r.stdout


def test_committed_loopback_profiles_run_from_a_bare_clone():
    r = sh('run.bash', '--print', RR_NET_MODE='loopback')
    assert r.returncode == 0, r.stderr
    assert 'fastdds_loopback.xml' in r.stdout
