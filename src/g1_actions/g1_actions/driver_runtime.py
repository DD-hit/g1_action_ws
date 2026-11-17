"""Require the action workspace's executable AND loaded driver library."""
import os
from pathlib import Path


def driver_config_match(argv, expected):
    if not argv or Path(argv[0]).name != 'app_run_component' or 'brsd_upbody_control' not in Path(argv[0]).parts:
        return None
    configs = [a.split('=', 1)[1] for a in argv if a.startswith('--component_config=')]
    return len(configs) == 1 and Path(configs[0]).resolve() == Path(expected).resolve()


def driver_runtime_match(proc, prefix):
    prefix = Path(prefix)
    try:
        expected_exe = (prefix / 'lib/brsd_upbody_control/app_run_component').resolve(strict=True)
        expected_library = (prefix / 'lib/libbrsd_upbody_control.so').resolve(strict=True)
        if (proc / 'exe').resolve(strict=True) != expected_exe:
            return False
        for row in (proc / 'maps').read_text().splitlines():
            fields = row.split(maxsplit=5)
            if len(fields) != 6 or fields[5].endswith(' (deleted)'):
                continue
            if Path(fields[5]).name == 'libbrsd_upbody_control.so' and Path(fields[5]).resolve() == expected_library:
                return True
    except (OSError, RuntimeError):
        return False
    return False


def verify_driver_runtime(motion):
    if not motion.get('require_business_driver', True):
        return
    expected = Path(motion['driver_config']).resolve()
    prefix = motion.get('driver_install_prefix')
    matches = []
    domain = os.environ.get('ROS_DOMAIN_ID', '0')
    for proc in Path('/proc').glob('[0-9]*'):
        try:
            argv = (proc / 'cmdline').read_bytes().decode().split('\0')
            match = driver_config_match(argv, expected)
            if match is None:
                continue
            env = dict(x.split(b'=', 1) for x in (proc / 'environ').read_bytes().split(b'\0') if b'=' in x)
            if env.get(b'ROS_DOMAIN_ID', b'0').decode() != domain:
                continue
        except (OSError, UnicodeError):
            continue
        matches.append(bool(match and (prefix is None or driver_runtime_match(proc, prefix))))
    if matches != [True]:
        raise RuntimeError('独立 driver 校验失败：配置、可执行文件或已加载动态库不属于本工作区；请退出旧服务后运行 bash scripts/g1.sh serve')
