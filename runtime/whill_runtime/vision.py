"""RITnet ONNX segmentation and original native PuReST; camera geometry in full frame."""
import ctypes
from pathlib import Path
import time

import cv2
import numpy as np
import onnx
import onnxruntime as ort

ROOT = Path(__file__).resolve().parents[2]


class Segmenter:
    def __init__(self):
        options = ort.SessionOptions()
        options.intra_op_num_threads = 4
        source = ROOT / 'projects/EYETLE/models/ritnet/ritnet_320x240.onnx'
        model = onnx.load(str(source))
        # The exported two-channel tensor packs class IDs and max confidence.
        # Expose the original four class logits upstream of its postprocessing.
        softmax = next(n for n in model.graph.node if n.op_type == 'Softmax')
        del model.graph.output[:]
        model.graph.output.append(onnx.helper.make_tensor_value_info(softmax.input[0], onnx.TensorProto.FLOAT, [1,4,240,320]))
        self.session = ort.InferenceSession(model.SerializeToString(),
                                            sess_options=options, providers=['CPUExecutionProvider'])
        self.input = self.session.get_inputs()[0]

    def run(self, gray):
        image = cv2.resize(gray, (320, 240))
        gamma = (255 * (np.linspace(0,1,256)**.8)).astype(np.uint8)
        image = cv2.LUT(image,gamma)
        image = cv2.createCLAHE(clipLimit=1.5,tileGridSize=(8,8)).apply(image)
        image = image.astype(np.float32)/127.5-1.
        logits = self.session.run(None, {self.input.name: image[None, None]})[0]
        if logits.ndim != 4 or logits.shape[1] != 4:
            raise RuntimeError(f'Unexpected RITnet output: {logits.shape}')
        logits = logits[0]
        exp = np.exp(logits - logits.max(axis=0))
        probability = exp / exp.sum(axis=0)
        return probability.argmax(axis=0).astype(np.uint8), probability


class NativePuReST:
    def __init__(self):
        self.lib = ctypes.CDLL(str(ROOT / '.local/lib/libwhill_purest.so'))
        self.lib.purest_create.restype = ctypes.c_void_p
        self.lib.purest_destroy.argtypes = [ctypes.c_void_p]
        self.lib.purest_reset.argtypes = [ctypes.c_void_p]
        self.lib.purest_run.argtypes = [ctypes.c_void_p, ctypes.c_void_p] + [ctypes.c_int] * 7 + [ctypes.POINTER(ctypes.c_float)]
        self.handle = self.lib.purest_create()
        self.roi = None
        self.last = 0.

    def run(self, gray, roi, now):
        if roi is None:
            self.lib.purest_reset(self.handle)
            return None
        if roi != self.roi or not 0 < now - self.last <= .25:
            self.lib.purest_reset(self.handle)
        self.roi, self.last = roi, now
        gray = np.ascontiguousarray(gray)
        result = (ctypes.c_float * 3)()
        ok = self.lib.purest_run(self.handle, gray.ctypes.data, gray.shape[1], gray.shape[0],
                                 gray.strides[0], *roi, result)
        return tuple(result) if ok else None

    def close(self):
        if self.handle:
            self.lib.purest_destroy(self.handle)
            self.handle = None


class EyeDetector:
    def __init__(self, segmenter):
        self.segmenter = segmenter
        self.purest = NativePuReST()
        self.open_reference = None
        self.open_at = -float('inf')

    def run(self, frame, now):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        labels, probability = self.segmenter.run(gray)
        eye_mask = ((labels > 0) & (probability.max(axis=0) > .65)).astype(np.uint8)
        points = cv2.findNonZero(eye_mask)
        roi = None
        if points is not None and len(points) > labels.size * .01:
            x, y, w, h = cv2.boundingRect(points)
            sx, sy = gray.shape[1] / 320., gray.shape[0] / 240.
            roi = (max(0, int((x - 5)*sx)), max(0, int((y - 5)*sy)),
                   min(gray.shape[1], int((w + 10)*sx)), min(gray.shape[0], int((h + 10)*sy)))
        pupil_mask = ((labels == 3) & (probability[3] > .65)).astype(np.uint8)
        n, components, stats, centers = cv2.connectedComponentsWithStats(pupil_mask)
        candidates = []
        margin = max(2, int(min(labels.shape)*.025))
        for i in range(1, n):
            x, y, w, h, area = stats[i]
            if not 8 <= area <= labels.size * .08 or x < margin or y < margin or x+w >= 320-margin or y+h >= 240-margin:
                continue
            mask = (components == i).astype(np.uint8)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if len(contours[0]) < 5:
                continue
            axes = cv2.fitEllipse(contours[0])[1]
            if min(axes) / max(axes) < .25:
                continue
            ring = (cv2.dilate(mask, np.ones((7, 7), np.uint8)) > 0) & (mask == 0)
            if not ring.any() or np.mean(labels[ring] == 2) < .35:
                continue
            quality = float(probability[3][components == i].mean())
            if quality >= .85:
                candidates.append((area, centers[i], quality))
        candidates.sort(key=lambda c: c[0], reverse=True)
        pupil = candidates[0] if candidates and (len(candidates) == 1 or candidates[1][0] < candidates[0][0]*.7) else None
        state = 'unknown'
        feature = [None, None]
        quality = [0., 0.]
        if pupil is not None and roi is not None:
            feature[0] = float(pupil[1][0] / 320)
            quality[0] = pupil[2]
            state = 'open'
            resized = cv2.resize(gray, (320, 240)).astype(np.float32) / 255.
            area = int(eye_mask.sum())
            self.open_reference = (area, resized, eye_mask.copy())
            self.open_at = now
        elif self.open_reference is not None and now-self.open_at < 1.5:
            area, previous, previous_mask = self.open_reference
            resized = cv2.resize(gray, (320, 240)).astype(np.float32)/255.
            surrounds = cv2.dilate(previous_mask, np.ones((21,21), np.uint8)) == 0
            surrounds[:6] = surrounds[-6:] = False
            surrounds[:, :6] = surrounds[:, -6:] = False
            confident = float(probability.max(axis=0).mean()) > .85
            if (eye_mask.sum() < area*.18 and confident and resized.std() > .025
                    and surrounds.mean() > .2 and np.abs(resized-previous)[surrounds].mean() < .09):
                state = 'closed'
        pure = self.purest.run(gray, roi, now)
        if pure is not None:
            x, y, q = pure
            consistent = pupil is None or np.linalg.norm(np.array([x/gray.shape[1], y/gray.shape[0]]) - pupil[1]/[320,240]) <= .06
            if consistent and 0 <= x < gray.shape[1] and 0 <= y < gray.shape[0]:
                feature[1], quality[1] = x/gray.shape[1], q
        return dict(state=state, features=feature, quality=quality, roi=roi)
