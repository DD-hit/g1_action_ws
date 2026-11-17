from pathlib import Path
from setuptools import find_packages, setup

assets = [
    ('share/ament_index/resource_index/packages', ['resource/g1_actions']),
    ('share/g1_actions', ['package.xml']),
]
for folder in ('config', 'schemas', 'models', 'robot', 'native', 'scripts', 'test'):
    files = sorted(p for p in Path(folder).rglob('*') if p.is_file()
                   and '__pycache__' not in p.parts
                   and p.suffix not in ('.pyc', '.pyo', '.bak')
                   and '.before_' not in p.name
                   and p.name != '.DS_Store' and not p.name.startswith('._'))
    for directory in sorted({p.parent for p in files}):
        assets.append((str(Path('share/g1_actions') / directory),
                       [str(p) for p in files if p.parent == directory]))
setup(
    name='g1_actions', version='0.1.0', packages=find_packages(),
    data_files=assets, install_requires=['setuptools'],
    entry_points={'console_scripts': [
        'actions = g1_actions.client:main',
        'elevator_vision = g1_actions.actions.elevator.vision_node:main',
        'elevator_press = g1_actions.actions.elevator.task_node:main',
        'elevator_model_check = g1_actions.actions.elevator.robot_model:main',
        'elevator_calibrate = g1_actions.actions.elevator.calibration:main',
    ]},
)
