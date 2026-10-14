"""Read resource counters only for an owned AMCL process and its descendants."""

from __future__ import annotations

import os
from pathlib import Path

__all__ = ["process_tree_counters"]


def process_tree_counters(pid: int):
    pending = [pid]
    ticks = 0
    rss = 0
    while pending:
        current = pending.pop()
        try:
            fields = Path(f"/proc/{current}/stat").read_text().split(")", 1)[1].split()
            ticks += int(fields[11]) + int(fields[12])
            rss += int(fields[21]) * os.sysconf("SC_PAGE_SIZE")
            children = (
                Path(f"/proc/{current}/task/{current}/children").read_text().split()
            )
            pending.extend(map(int, children))
        except (FileNotFoundError, ProcessLookupError):
            continue
    return ticks / os.sysconf("SC_CLK_TCK"), rss / 1024**2
