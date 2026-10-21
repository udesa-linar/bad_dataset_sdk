"""Download the Buenos Aires Dataset, or only the parts you need, from Hugging Face.

    from badataset.download import download
    download('/data/buenos_aires', sequences=['loop_caba'], groups=['imu_gps', 'ground_truth', 'tf_static'])

or from the shell:

    python -m badataset.download /data/buenos_aires --sequences loop_caba --groups imu_gps ground_truth tf_static
    python -m badataset.download /data/buenos_aires --calib            # only the calibration recordings

The files keep the release layout (<root>/<sequence>/split_XX/<group>/...), which is what Dataset(root) reads.
A download that is interrupted resumes where it stopped. Needs huggingface_hub: pip install -e ".[download]".
"""
import argparse

from .reader import SEQUENCES

REPO_ID = 'udesa-linar/buenos-aires-dataset'
GROUPS = ['cameras', 'lidar', 'imu_gps', 'dataspeed', 'wheel', 'microphone', 'ground_truth', 'tf_static']


def patterns(sequences=None, groups=None, splits=None, calib=False):
    """allow_patterns of the requested parts: every sequence/group/split when None; calib adds calib/"""
    sequences, groups = sequences or SEQUENCES, groups or GROUPS
    for name, values, known in (('sequence', sequences, SEQUENCES), ('group', groups, GROUPS)):
        unknown = sorted(set(values) - set(known))
        if unknown:
            raise ValueError(f'unknown {name} {unknown}; choose from {known}')
    out = [f'{s}/{sp}/{g}/*' for s in sequences for sp in (splits or ['split_*']) for g in groups]
    return out + ['calib/*'] if calib else out


def download(root, sequences=None, groups=None, splits=None, calib=False, repo_id=REPO_ID, revision=None):
    """download the requested parts of the release into root and return root.

    sequences: e.g. ['caba']; groups: e.g. ['lidar', 'tf_static']; splits: e.g. ['split_03']; None = all.
    calib=True also downloads calib/ (the calibration recordings); calib=True with sequences=[] downloads only those.
    revision: a tag or commit of the dataset repository, to pin a version of the release."""
    try:
        from huggingface_hub import snapshot_download
    except ImportError:
        raise SystemExit('download needs huggingface_hub: pip install -e ".[download]"') from None
    allow = ['calib/*'] if calib and sequences == [] else patterns(sequences, groups, splits, calib)
    allow += ['README.md']
    return snapshot_download(repo_id=repo_id, repo_type='dataset', revision=revision, local_dir=root,
                             allow_patterns=allow)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('root', help='folder to download into')
    ap.add_argument('--sequences', nargs='*', choices=SEQUENCES, help='default: all')
    ap.add_argument('--groups', nargs='*', choices=GROUPS, help='default: all')
    ap.add_argument('--splits', nargs='*', help='e.g. split_03; default: all')
    ap.add_argument('--calib', action='store_true', help='also the calibration recordings (calib/); alone: only them')
    ap.add_argument('--revision', help='tag or commit of the dataset repository')
    ap.add_argument('--repo-id', default=REPO_ID)
    a = ap.parse_args(argv)
    sequences = [] if a.calib and not (a.sequences or a.groups or a.splits) else a.sequences
    print(download(a.root, sequences, a.groups, a.splits, a.calib, a.repo_id, a.revision))


if __name__ == '__main__':
    main()
