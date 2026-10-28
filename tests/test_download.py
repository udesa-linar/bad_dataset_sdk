"""Tests of the download helper that need neither the data nor the network (snapshot_download is replaced)."""
import sys
import types

import pytest

from badataset import download as D


class RepositoryNotFoundError(Exception):
    pass


def fake_hub(monkeypatch, snapshot):
    hub, errors = types.ModuleType('huggingface_hub'), types.ModuleType('huggingface_hub.errors')
    hub.snapshot_download, errors.RepositoryNotFoundError = snapshot, RepositoryNotFoundError
    monkeypatch.setitem(sys.modules, 'huggingface_hub', hub)
    monkeypatch.setitem(sys.modules, 'huggingface_hub.errors', errors)


@pytest.fixture
def calls(monkeypatch):
    seen = []
    fake_hub(monkeypatch, lambda **kw: seen.append(kw) or kw['local_dir'])
    return seen


def test_patterns_select_parts():
    p = D.patterns(['caba'], ['lidar', 'tf_static'], ['split_03'])
    assert p == ['caba/split_03/lidar/*', 'caba/split_03/tf_static/*']
    assert len(D.patterns()) == len(D.SEQUENCES) * len(D.GROUPS)
    assert D.patterns(['caba'], ['lidar'], calib=True)[-1] == 'calib/*'


def test_patterns_reject_unknown():
    with pytest.raises(ValueError):
        D.patterns(['nowhere'])
    with pytest.raises(ValueError):
        D.patterns(groups=['radar'])


def test_download_arguments(calls):
    assert D.download('/data', ['loop_caba'], ['imu_gps']) == '/data'
    kw = calls[-1]
    assert kw['repo_id'] == D.REPO_ID and kw['repo_type'] == 'dataset' and kw['local_dir'] == '/data'
    assert kw['allow_patterns'] == ['loop_caba/split_*/imu_gps/*', 'README.md']


def test_cli_calib_only(calls):
    D.main(['/data', '--calib'])
    assert calls[-1]['allow_patterns'] == ['calib/*', 'README.md']
    D.main(['/data', '--sequences', 'caba', '--calib', '--revision', 'v1.0'])
    assert 'calib/*' in calls[-1]['allow_patterns'] and calls[-1]['revision'] == 'v1.0'
    assert len(calls[-1]['allow_patterns']) == len(D.GROUPS) + 2


def test_not_published(monkeypatch):
    def missing(**kw):
        raise RepositoryNotFoundError
    fake_hub(monkeypatch, missing)
    with pytest.raises(SystemExit, match='not available for download yet'):
        D.download('/data', ['caba'])
    with pytest.raises(SystemExit, match='Hugging Face'):
        D.main(['/data', '--calib'])
