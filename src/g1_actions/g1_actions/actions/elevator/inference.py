import ctypes as C
import time
import cv2
import numpy as np
from .config import resource
from .geometry import resolve_conflicts


class TensorRT:
    def __init__(self, library, engine):
        self.lib = C.CDLL(str(library))
        self.lib.ele_trt_error.restype = C.c_char_p
        self.lib.ele_trt_create.argtypes = [C.c_char_p]
        self.lib.ele_trt_create.restype = C.c_void_p
        self.lib.ele_trt_shape.argtypes = [C.c_void_p, C.c_int, C.POINTER(C.c_int), C.c_int]
        self.lib.ele_trt_infer.argtypes = [C.c_void_p, C.c_void_p, C.c_size_t, C.c_void_p, C.c_size_t]
        self.lib.ele_trt_destroy.argtypes = [C.c_void_p]
        self.handle = self.lib.ele_trt_create(str(engine).encode())
        if not self.handle:
            raise RuntimeError(self.lib.ele_trt_error().decode())
        self.input_shape, self.output_shape = self.shape(0), self.shape(1)
        self.output = np.empty(self.output_shape, dtype=np.float32)

    def shape(self, output):
        dims = (C.c_int*8)()
        n = self.lib.ele_trt_shape(self.handle, output, dims, 8)
        if n <= 0:
            raise RuntimeError('Invalid TensorRT tensor shape')
        return tuple(dims[:n])

    def run(self, value):
        value = np.ascontiguousarray(value, dtype=np.float32)
        if tuple(value.shape) != self.input_shape:
            raise ValueError('Input dimensions differ from compiled engine')
        if self.lib.ele_trt_infer(self.handle, value.ctypes.data, value.size,
                                  self.output.ctypes.data, self.output.size):
            raise RuntimeError(self.lib.ele_trt_error().decode())
        return self.output.copy()

    def close(self):
        if self.handle:
            self.lib.ele_trt_destroy(self.handle)
            self.handle = None


class Detector:
    def __init__(self, config):
        self.options = config['detector']
        self.size = self.options['imgsz']
        self.backend = self.options['backend']
        if self.backend == 'tensorrt':
            self.engine = TensorRT(self.options['native_library'], resource(config, self.options['engine']))
            if self.engine.input_shape != (1, 3, self.size, self.size):
                raise ValueError('Configured imgsz does not match TensorRT engine')
            # Initialize lazy CUDA/cuDNN kernels before accepting timestamped
            # camera frames; startup latency must not turn a frame into a stale target.
            warmup = np.zeros(self.engine.input_shape, dtype=np.float32)
            for _ in range(3):
                self.engine.run(warmup)
        elif self.backend == 'onnx_cpu':
            import onnxruntime as ort
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = 4
            self.engine = ort.InferenceSession(str(resource(config, self.options['onnx'])),
                                              sess_options=opts, providers=['CPUExecutionProvider'])
        else:
            raise ValueError('backend must be tensorrt or onnx_cpu')

    def predict(self, bgr):
        h, w = bgr.shape[:2]
        scale = min(self.size/w, self.size/h)
        nw, nh = int(round(w*scale)), int(round(h*scale))
        dx, dy = (self.size-nw)//2, (self.size-nh)//2
        canvas = np.full((self.size, self.size, 3), 114, dtype=np.uint8)
        canvas[dy:dy+nh, dx:dx+nw] = cv2.resize(bgr, (nw, nh))
        value = np.ascontiguousarray(canvas[:, :, ::-1].transpose(2, 0, 1)[None], dtype=np.float32)/255
        start = time.perf_counter()
        if self.backend == 'tensorrt':
            output = self.engine.run(value)
        else:
            output = self.engine.run(None, {self.engine.get_inputs()[0].name: value})[0]
        elapsed = (time.perf_counter()-start)*1000
        if output.ndim != 3 or output.shape[-1] != 6:
            raise RuntimeError(f'Expected YOLOv10 end-to-end [1,N,6] output, got {output.shape}')
        detections = []
        for x1, y1, x2, y2, confidence, class_id in output[0]:
            cls = int(round(float(class_id)))
            if cls not in (265, 366) or confidence < self.options['confidence']:
                continue
            box = [(x1-dx)/scale, (y1-dy)/scale, (x2-dx)/scale, (y2-dy)/scale]
            box = np.clip(box, [0, 0, 0, 0], [w-1, h-1, w-1, h-1]).tolist()
            if box[2] <= box[0] or box[3] <= box[1]:
                continue
            detections.append({'class_id': cls, 'direction': 'up' if cls == 366 else 'down',
                               'confidence': float(confidence), 'xyxy': box,
                               'center_xy': [(box[0]+box[2])/2, (box[1]+box[3])/2]})
        return resolve_conflicts(detections, self.options['confidence'], self.options['conflict_iou'],
                                 self.options['conflict_margin']), elapsed

    def close(self):
        if self.backend == 'tensorrt':
            self.engine.close()
