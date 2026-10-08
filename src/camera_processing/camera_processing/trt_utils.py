#!/usr/bin/env python3
"""
Shared TensorRT engine wrapper for camera_processing's detector nodes.

UNTESTED ON REAL HARDWARE - written against the standard TensorRT 8.x
Python API (binding-index based: execute_async_v2, get_binding_*), which is
what JetPack 6.0 ships (TensorRT 8.6.x). This dev machine has no GPU, no
TensorRT, and no PyCUDA installed, so this could only be validated by
inspection/pattern-matching against TensorRT's documented API, not by
actually running it - unlike everything else in this workspace, which was
verified live before being handed over. Test this on the Orin and report
back the exact error if anything breaks; the most likely failure points are
noted inline.

Engines are expected to have been built with fixed (non-dynamic) shapes,
matching how `trtexec --onnx=... --saveEngine=... --fp16` (or without
--fp16 for full precision) was actually run here - no explicit optimization
profile / dynamic shape handling is implemented.
"""
import numpy as np


class TrtEngine:
    """Loads a .trt/.engine file and runs inference, returning outputs by name.

    Requires `tensorrt` and `pycuda` to be installed (Jetson-side only -
    neither is expected to exist on a generic x86_64 dev machine). Import is
    deferred to __init__ so importing this module doesn't fail on machines
    without them.
    """

    def __init__(self, engine_path, logger=None):
        import tensorrt as trt
        import pycuda.driver as cuda
        import pycuda.autoinit  # noqa: F401 - initializes a CUDA context on import

        self._cuda = cuda
        self._log = logger.info if logger else print

        trt_logger = trt.Logger(trt.Logger.WARNING)
        with open(engine_path, 'rb') as f, trt.Runtime(trt_logger) as runtime:
            self.engine = runtime.deserialize_cuda_engine(f.read())
        if self.engine is None:
            raise RuntimeError(
                f'Failed to deserialize TensorRT engine: {engine_path}. Engines are '
                'tied to the exact TensorRT version/GPU they were built on - if this '
                'engine was built elsewhere, re-run trtexec on this Orin.'
            )
        self.context = self.engine.create_execution_context()
        self.stream = cuda.Stream()

        self.input_name = None
        self.input_shape = None
        self._bindings = []
        self._host_buffers = {}   # name -> pagelocked host array
        self._device_buffers = {}  # name -> device pointer
        self._output_names = []
        self._output_shapes = {}

        for i in range(self.engine.num_bindings):
            name = self.engine.get_binding_name(i)
            shape = tuple(self.engine.get_binding_shape(i))
            dtype = trt.nptype(self.engine.get_binding_dtype(i))
            size = int(np.prod(shape))
            host_mem = cuda.pagelocked_empty(size, dtype)
            device_mem = cuda.mem_alloc(host_mem.nbytes)
            self._bindings.append(int(device_mem))
            self._host_buffers[name] = host_mem
            self._device_buffers[name] = device_mem

            if self.engine.binding_is_input(i):
                if self.input_name is not None:
                    raise RuntimeError(
                        f'TrtEngine only supports single-input engines; found a second '
                        f'input binding "{name}" alongside "{self.input_name}".'
                    )
                self.input_name = name
                self.input_shape = shape
            else:
                self._output_names.append(name)
                self._output_shapes[name] = shape

        self._log(
            f'TensorRT engine loaded: {engine_path} | input: {self.input_name} '
            f'{self.input_shape} | outputs: {self._output_names}'
        )

    def infer(self, blob):
        """blob: numpy array matching self.input_shape (batch dim included). Returns
        dict of {output_name: numpy array in its declared shape}."""
        cuda = self._cuda
        host_in = self._host_buffers[self.input_name]
        np.copyto(host_in, np.ascontiguousarray(blob, dtype=host_in.dtype).ravel())
        cuda.memcpy_htod_async(self._device_buffers[self.input_name], host_in, self.stream)

        ok = self.context.execute_async_v2(bindings=self._bindings, stream_handle=self.stream.handle)
        if not ok:
            raise RuntimeError('TensorRT execute_async_v2 returned False')

        for name in self._output_names:
            cuda.memcpy_dtoh_async(
                self._host_buffers[name], self._device_buffers[name], self.stream)
        self.stream.synchronize()

        return {
            name: self._host_buffers[name].reshape(self._output_shapes[name]).copy()
            for name in self._output_names
        }

    def get_output(self, outputs, *candidate_names, index_fallback=None):
        """Look up one output by trying each candidate name in order (ONNX export
        tools sometimes rename tensors), falling back to positional index if given
        and no name matched - engines normally preserve the ONNX graph's own
        output tensor names, but this isn't guaranteed across every export path."""
        for name in candidate_names:
            if name in outputs:
                return outputs[name]
        if index_fallback is not None and index_fallback < len(self._output_names):
            return outputs[self._output_names[index_fallback]]
        raise KeyError(
            f'None of {candidate_names} found in engine outputs {list(outputs.keys())}'
        )
