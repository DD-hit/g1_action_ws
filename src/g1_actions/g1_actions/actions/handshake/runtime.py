"""Compatibility entry point: the shared backend owns all actions."""
import os
from pathlib import Path


def main():
    project = Path(__file__).resolve().parents[5]
    os.execvp('bash', ['bash', str(project/'scripts/g1.sh'), 'serve'])


if __name__ == '__main__':
    main()
