#!/usr/bin/env python3
"""Manage camera and vision together, with complete descendant cleanup."""
from pathlib import Path
from managed_service import supervise

if __name__=='__main__':
    root=Path(__file__).resolve().parents[1]
    raise SystemExit(supervise('perception',[
        ['bash',str(root/'scripts/ele.sh'),'camera'],
        ['bash',str(root/'scripts/ele.sh'),'vision'],
    ]))
