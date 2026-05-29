# Field layout YAML. Same numbers as Unity FieldLayout.
# Override for a demo: FIELD_LAYOUT_PATH=/path/to/field_layout_demo.yaml

import os
from pathlib import Path

_LAYOUT = None


def _repo_root():
    return Path(__file__).resolve().parents[3]


def layout_path():
    override = os.environ.get('FIELD_LAYOUT_PATH', '').strip()
    if override:
        return Path(override)
    try:
        from ament_index_python.packages import get_package_share_directory
        share = Path(get_package_share_directory('drone_system_pkg'))
        installed = share / 'config' / 'field_layout.yaml'
        if installed.is_file():
            return installed
    except Exception:
        pass
    return _repo_root() / 'config' / 'field_layout.yaml'


def load_layout():
    global _LAYOUT
    if _LAYOUT is not None:
        return _LAYOUT
    path = layout_path()
    if not path.is_file():
        raise FileNotFoundError(f'field layout not found: {path}')
    try:
        import yaml
    except ImportError as exc:
        raise ImportError('Install PyYAML: sudo apt install python3-yaml') from exc
    with path.open('r', encoding='utf-8') as f:
        _LAYOUT = yaml.safe_load(f)
    return _LAYOUT


def bird_count():
    birds = load_layout().get('birds', [])
    return len(birds) if birds else 1


def obstacle_list():
    # Each entry: x, y, radius (m), optional name.
    return list(load_layout().get('obstacles', []))


def obstacle_count():
    obs = obstacle_list()
    return len(obs) if obs else 0
