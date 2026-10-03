from pathlib import Path

from scripts.host_cpu_isolation import process_start_ticks


def test_process_start_ticks_reads_current_process():
    import os
    assert isinstance(process_start_ticks(os.getpid()), int)
    assert process_start_ticks(os.getpid()) > 0
