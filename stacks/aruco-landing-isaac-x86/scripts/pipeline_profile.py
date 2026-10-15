"""Opt-in CPU wall/copy ledger; does not add CUDA synchronization or copies."""
from contextlib import contextmanager
import time

class PipelineProfile:
    def __init__(self, enabled=False):
        self.enabled=bool(enabled);self.wall={};self.transfers={};self.counts={}

    def add(self,name,seconds):
        if self.enabled:
            item=self.wall.setdefault(name,{'calls':0,'wall_s':0.})
            item['calls']+=1;item['wall_s']+=float(seconds)

    @contextmanager
    def stage(self,name):
        if not self.enabled:
            yield;return
        start=time.perf_counter()
        try:yield
        finally:self.add(name,time.perf_counter()-start)

    def copy(self,name,tensor,destination,operation):
        if not self.enabled:return operation()
        start=time.perf_counter();result=operation()
        item=self.transfers.setdefault(name,{'calls':0,'bytes':0,'wall_s':0.,'direction':destination})
        item['calls']+=1;item['bytes']+=tensor.numel()*tensor.element_size()
        item['wall_s']+=time.perf_counter()-start
        return result

    def construct(self,name,byte_count,destination,operation):
        """Count a visible host-data tensor construction; not hidden driver copies."""
        if not self.enabled:return operation()
        start=time.perf_counter();result=operation()
        item=self.transfers.setdefault(name,{'calls':0,'bytes':0,'wall_s':0.,'direction':destination})
        item['calls']+=1;item['bytes']+=int(byte_count);item['wall_s']+=time.perf_counter()-start
        return result

    def count(self,name):
        if self.enabled:self.counts[name]=self.counts.get(name,0)+1

    def report(self):
        return dict(enabled=self.enabled,wall=self.wall,explicit_tensor_copies=self.transfers,counts=self.counts,
            scope='Explicit authored PyTorch copies only; RTX/PhysX/Fabric/driver internal communication uninstrumented',
            wall_convention='CPU elapsed wall; async enqueue and subsequent wait can be charged to later stage; no new GPU synchronization',
            additivity='Transfer timers are nested inside named wall stages; do not add both. Existing timing totals remain separate inclusive ledgers.')
