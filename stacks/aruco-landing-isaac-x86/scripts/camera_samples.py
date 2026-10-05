"""Opt-in capture evidence for same-image CPU/GPU detector audits."""
import hashlib
import cv2
import numpy as np
import torch


class CameraSamples:
    def __init__(self, store, dictionary, interval, error_threshold_m=.3):
        self.store, self.dictionary, self.interval = store, dictionary, interval
        self.error_threshold_m = error_threshold_m
        self.entries, self.extra_batches = [], 0
        self.directory = store.directory/'camera-samples'
        self.directory.mkdir(exist_ok=False)

    def capture(self, batch, rgb, truth, observations, ids):
        estimates=[]
        maximum_error=0.
        for gt, observation in zip(truth, observations):
            estimate=None if observation is None else np.linalg.inv(observation['camera_from_pad'])
            estimates.append(estimate)
            if estimate is not None:
                maximum_error=max(maximum_error,float(np.linalg.norm(estimate[:3,3]-gt[:3,3])))
        periodic=batch % self.interval == 0
        extra=maximum_error>=self.error_threshold_m and self.extra_batches<16
        if not periodic and not extra: return
        if not periodic: self.extra_batches+=1
        gray=((rgb[...,:3].to(torch.int32)*torch.tensor([77,150,29],device=rgb.device,dtype=torch.int32))
              .sum(-1,dtype=torch.int32)>>8).to(torch.uint8).cpu().numpy()
        for env, image in enumerate(gray):
            path=self.directory/('sample-%06d-env-%03d.png' % (batch,env))
            if not cv2.imwrite(str(path),image,[cv2.IMWRITE_PNG_COMPRESSION,3]):
                raise OSError('failed to save camera audit image')
            self.entries.append(dict(path=str(path),dictionary=self.dictionary,
                file_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                gray_sha256=hashlib.sha256(image.tobytes()).hexdigest(),
                camera_batch=batch,environment=env,periodic=periodic,
                gt_pad_from_camera=truth[env].tolist(),main_gpu_ids=list(ids[env]),
                main_estimated_pad_from_camera=None if estimates[env] is None else estimates[env].tolist()))
        self.store._atomic(self.store.directory/'camera-corpus.json',self.entries)
