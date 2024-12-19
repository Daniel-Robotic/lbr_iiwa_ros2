import contextlib
import decimal
import inspect
import math
import pickle
import re
from pathlib import Path
from unittest import mock

import numpy as np
import PIL.Image
import pytest

import torch
import torchvision.transforms.v2 as transforms
from common_utils import (
    assert_equal,
    assert_no_warnings,
    cache,
    cpu_and_cuda,
    freeze_rng_state,
    ignore_jit_no_profile_information_warning,
    make_bounding_boxes,
    make_detection_mask,
    make_image,
    make_image_pil,
    make_image_tensor,
    make_segmentation_mask,
    make_video,
    make_video_tensor,
    needs_cuda,
    set_rng_seed,
)

from torch import nn
from torch.testing import assert_close
from torch.utils._pytree import tree_map
from torch.utils.data import DataLoader, default_collate
from torchvision import tv_tensors

from torchvision.transforms._functional_tensor import _max_value as get_max_value
from torchvision.transforms.functional import pil_modes_mapping
from torchvision.transforms.v2 import functional as F
from torchvision.transforms.v2.functional._utils import _get_kernel, _register_kernel_internal


@pytest.fixture(autouse=True)
def fix_rng_seed():
    set_rng_seed(0)
    yield


def _to_tolerances(maybe_tolerance_dict):
    if not isinstance(maybe_tolerance_dict, dict):
        return dict(rtol=None, atol=None)

    tolerances = dict(rtol=0, atol=0)
    tolerances.update(maybe_tolerance_dict)
    return tolerances


def _check_kernel_cuda_vs_cpu(kernel, input, *args, rtol, atol, **kwargs):
    """Checks if the kernel produces closes results for inputs on GPU and CPU."""
    if input.device.type != "cuda":
        return

    input_cuda = input.as_subclass(torch.Tensor)
    input_cpu = input_cuda.to("cpu")

    with freeze_rng_state():
        actual = kernel(input_cuda, *args, **kwargs)
    with freeze_rng_state():
        expected = kernel(input_cpu, *args, **kwargs)

    assert_close(actual, expected, check_device=False, rtol=rtol, atol=atol)


@cache
def _script(obj):
    try:
        return torch.jit.script(obj)
    except Exception as error:
        name = getattr(obj, "__name__", obj.__class__.__name__)
        raise AssertionError(f"Trying to `torch.jit.script` '{name}' raised the error above.") from error


def _check_kernel_scripted_vs_eager(kernel, input, *args, rtol, atol, **kwargs):
    """Checks if the kernel is scriptable and if the scripted output is close to the eager one."""
    if input.device.type != "cpu":
        return

    kernel_scripted = _script(kernel)

    input = input.as_subclass(torch.Tensor)
    with ignore_jit_no_profile_information_warning():
        actual = kernel_scripted(input, *args, **kwargs)
    expected = kernel(input, *args, **kwargs)

    assert_close(actual, expected, rtol=rtol, atol=atol)


def _check_kernel_batched_vs_unbatched(kernel, input, *args, rtol, atol, **kwargs):
    """Checks if the kernel produces close results for batched and unbatched inputs."""
    unbatched_input = input.as_subclass(torch.Tensor)

    for batch_dims in [(2,), (2, 1)]:
        repeats = [*batch_dims, *[1] * input.ndim]

        actual = kernel(unbatched_input.repeat(repeats), *args, **kwargs)

        expected = kernel(unbatched_input, *args, **kwargs)
        # We can't directly call `.repeat()` on the output, since some kernel also return some additional metadata
        if isinstance(expected, torch.Tensor):
            expected = expected.repeat(repeats)
        else:
            tensor, *metadata = expected
            expected = (tensor.repeat(repeats), *metadata)

        assert_close(actual, expected, rtol=rtol, atol=atol)

    for degenerate_batch_dims in [(0,), (5, 0), (0, 5)]:
        degenerate_batched_input = torch.empty(
            degenerate_batch_dims + input.shape, dtype=input.dtype, device=input.device
        )

        output = kernel(degenerate_batched_input, *args, **kwargs)
        # Most kernels just return a tensor, but some also return some additional metadata
        if not isinstance(output, torch.Tensor):
            output, *_ = output

        assert output.shape[: -input.ndim] == degenerate_batch_dims


def check_kernel(
    kernel,
    input,
    *args,
    check_cuda_vs_cpu=True,
    check_scripted_vs_eager=True,
    check_batched_vs_unbatched=True,
    expect_same_dtype=True,
    **kwargs,
):
    initial_input_version = input._version

    output = kernel(input.as_subclass(torch.Tensor), *args, **kwargs)
    # Most kernels just return a tensor, but some also return some additional metadata
    if not isinstance(output, torch.Tensor):
        output, *_ = output

    # check that no inplace operation happened
    assert input._version == initial_input_version

    if expect_same_dtype:
        assert output.dtype == input.dtype
    assert output.device == input.device

    if check_cuda_vs_cpu:
        _check_kernel_cuda_vs_cpu(kernel, input, *args, **kwargs, **_to_tolerances(check_cuda_vs_cpu))

    if check_scripted_vs_eager:
        _check_kernel_scripted_vs_eager(kernel, input, *args, **kwargs, **_to_tolerances(check_scripted_vs_eager))

    if check_batched_vs_unbatched:
        _check_kernel_batched_vs_unbatched(kernel, input, *args, **kwargs, **_to_tolerances(check_batched_vs_unbatched))


def _check_functional_scripted_smoke(functional, input, *args, **kwargs):
    """Checks if the functional can be scripted and the scripted version can be called without error."""
    if not isinstance(input, tv_tensors.Image):
        return

    functional_scripted = _script(functional)
    with ignore_jit_no_profile_information_warning():
        functional_scripted(input.as_subclass(torch.Tensor), *args, **kwargs)


def check_functional(functional, input, *args, check_scripted_smoke=True, **kwargs):
    unknown_input = object()
    with pytest.raises(TypeError, match=re.escape(str(type(unknown_input)))):
        functional(unknown_input, *args, **kwargs)

    with mock.patch("torch._C._log_api_usage_once", wraps=torch._C._log_api_usage_once) as spy:
        output = functional(input, *args, **kwargs)

        spy.assert_any_call(f"{functional.__module__}.{functional.__name__}")

    assert isinstance(output, type(input))

    if isinstance(input, tv_tensors.BoundingBoxes):
        assert output.format == input.format

    if check_scripted_smoke:
        _check_functional_scripted_smoke(functional, input, *args, **kwargs)


def check_functional_kernel_signature_match(functional, *, kernel, input_type):
    """Checks if the signature of the functional matches the kernel signature."""
    functional_params = list(inspect.signature(functional).parameters.values())[1:]
    kernel_params = list(inspect.signature(kernel).parameters.values())[1:]

    if issubclass(input_type, tv_tensors.TVTensor):
        # We filter out metadata that is implicitly passed to the functional through the input tv_tensor, but has to be
        # explicitly passed to the kernel.
        explicit_metadata = {
            tv_tensors.BoundingBoxes: {"format", "canvas_size"},
        }
        kernel_params = [param for param in kernel_params if param.name not in explicit_metadata.get(input_type, set())]

    functional_params = iter(functional_params)
    for functional_param, kernel_param in zip(functional_params, kernel_params):
        try:
            # In general, the functional parameters are a superset of the kernel parameters. Thus, we filter out
            # functional parameters that have no kernel equivalent while keeping the order intact.
            while functional_param.name != kernel_param.name:
                functional_param = next(functional_params)
        except StopIteration:
            raise AssertionError(
                f"Parameter `{kernel_param.name}` of kernel `{kernel.__name__}` "
                f"has no corresponding parameter on the functional `{functional.__name__}`."
            ) from None

        if issubclass(input_type, PIL.Image.Image):
            # PIL kernels often have more correct annotations, since they are not limited by JIT. Thus, we don't check
            # them in the first place.
            functional_param._annotation = kernel_param._annotation = inspect.Parameter.empty

        assert functional_param == kernel_param


def _check_transform_v1_compatibility(transform, input, *, rtol, atol):
    """If the transform defines the ``_v1_transform_cls`` attribute, checks if the transform has a public, static
    ``get_params`` method that is the v1 equivalent, the output is close to v1, is scriptable, and the scripted version
    can be called without error."""
    if type(input) is not torch.Tensor or isinstance(input, PIL.Image.Image):
        return

    v1_transform_cls = transform._v1_transform_cls
    if v1_transform_cls is None:
        return

    if hasattr(v1_transform_cls, "get_params"):
        assert type(transform).get_params is v1_transform_cls.get_params

    v1_transform = v1_transform_cls(**transform._extract_params_for_v1_transform())

    with freeze_rng_state():
        output_v2 = transform(input)

    with freeze_rng_state():
        output_v1 = v1_transform(input)

    assert_close(output_v2, output_v1, rtol=rtol, atol=atol)

    if isinstance(input, PIL.Image.Image):
        return

    _script(v1_transform)(input)


def check_transform(transform, input, check_v1_compatibility=True):
    pickle.loads(pickle.dumps(transform))

    output = transform(input)
    assert isinstance(output, type(input))

    if isinstance(input, tv_tensors.BoundingBoxes):
        assert output.format == input.format

    if check_v1_compatibility:
        _check_transform_v1_compatibility(transform, input, **_to_tolerances(check_v1_compatibility))


def transform_cls_to_functional(transform_cls, **transform_specific_kwargs):
    def wrapper(input, *args, **kwargs):
        transform = transform_cls(*args, **transform_specific_kwargs, **kwargs)
        return transform(input)

    wrapper.__name__ = transform_cls.__name__

    return wrapper


def param_value_parametrization(**kwargs):
    """Helper function to turn

    @pytest.mark.parametrize(
        ("param", "value"),
        ("a", 1),
        ("a", 2),
        ("a", 3),
        ("b", -1.0)
        ("b", 1.0)
    )

    into

    @param_value_parametrization(a=[1, 2, 3], b=[-1.0, 1.0])
    """
    return pytest.mark.parametrize(
        ("param", "value"),
        [(param, value) for param, values in kwargs.items() for value in values],
    )


def adapt_fill(value, *, dtype):
    """Adapt fill values in the range [0.0, 1.0] to the value range of the dtype"""
    if value is None:
        return value

    max_value = get_max_value(dtype)
    value_type = float if dtype.is_floating_point else int

    if isinstance(value, (int, float)):
        return value_type(value * max_value)
    elif isinstance(value, (list, tuple)):
        return type(value)(value_type(v * max_value) for v in value)
    else:
        raise ValueError(f"fill should be an int or float, or a list or tuple of the former, but got '{value}'.")


EXHAUSTIVE_TYPE_FILLS = [
    None,
    1,
    0.5,
    [1],
    [0.2],
    (0,),
    (0.7,),
    [1, 0, 1],
    [0.1, 0.2, 0.3],
    (0, 1, 0),
    (0.9, 0.234, 0.314),
]
CORRECTNESS_FILLS = [
    v for v in EXHAUSTIVE_TYPE_FILLS if v is None or isinstance(v, float) or (isinstance(v, list) and len(v) > 1)
]


# We cannot use `list(transforms.InterpolationMode)` here, since it includes some PIL-only ones as well
INTERPOLATION_MODES = [
    transforms.InterpolationMode.NEAREST,
    transforms.InterpolationMode.NEAREST_EXACT,
    transforms.InterpolationMode.BILINEAR,
    transforms.InterpolationMode.BICUBIC,
]


@contextlib.contextmanager
def assert_warns_antialias_default_value():
    with pytest.warns(UserWarning, match="The default value of the antialias parameter of all the resizing transforms"):
        yield


def reference_affine_bounding_boxes_helper(bounding_boxes, *, affine_matrix, new_canvas_size=None, clamp=True):
    format = bounding_boxes.format
    canvas_size = new_canvas_size or bounding_boxes.canvas_size

    def affine_bounding_boxes(bounding_boxes):
        dtype = bounding_boxes.dtype
        device = bounding_boxes.device

        # Go to float before converting to prevent precision loss in case of CXCYWH -> XYXY and W or H is 1
        input_xyxy = F.convert_bounding_box_format(
            bounding_boxes.to(dtype=torch.float64, device="cpu", copy=True),
            old_format=format,
            new_format=tv_tensors.BoundingBoxFormat.XYXY,
            inplace=True,
        )
        x1, y1, x2, y2 = input_xyxy.squeeze(0).tolist()

        points = np.array(
            [
                [x1, y1, 1.0],
                [x2, y1, 1.0],
                [x1, y2, 1.0],
                [x2, y2, 1.0],
            ]
        )
        transformed_points = np.matmul(points, affine_matrix.astype(points.dtype).T)

        output_xyxy = torch.Tensor(
            [
                float(np.min(transformed_points[:, 0])),
                float(np.min(transformed_points[:, 1])),
                float(np.max(transformed_points[:, 0])),
                float(np.max(transformed_points[:, 1])),
            ]
        )

        output = F.convert_bounding_box_format(
            output_xyxy, old_format=tv_tensors.BoundingBoxFormat.XYXY, new_format=format
        )

        if clamp:
            # It is important to clamp before casting, especially for CXCYWH format, dtype=int64
            output = F.clamp_bounding_boxes(
                output,
                format=format,
                canvas_size=canvas_size,
            )
        else:
            # We leave the bounding box as float64 so the caller gets the full precision to perform any additional
            # operation
            dtype = output.dtype

        return output.to(dtype=dtype, device=device)

    return tv_tensors.BoundingBoxes(
        torch.cat([affine_bounding_boxes(b) for b in bounding_boxes.reshape(-1, 4).unbind()], dim=0).reshape(
            bounding_boxes.shape
        ),
        format=format,
        canvas_size=canvas_size,
    )


# turns all warnings into errors for this module
pytestmark = pytest.mark.filterwarnings("error")


class TestResize:
    INPUT_SIZE = (17, 11)
    OUTPUT_SIZES = [17, [17], (17,), [12, 13], (12, 13)]

    def _make_max_size_kwarg(self, *, use_max_size, size):
        if use_max_size:
            if not (isinstance(size, int) or len(size) == 1):
                # This would result in an `ValueError`
                return None

            max_size = (size if isinstance(size, int) else size[0]) + 1
        else:
            max_size = None

        return dict(max_size=max_size)

    def _compute_output_size(self, *, input_size, size, max_size):
        if not (isinstance(size, int) or len(size) == 1):
            return tuple(size)

        if not isinstance(size, int):
            size = size[0]

        old_height, old_width = input_size
        ratio = old_width / old_height
        if ratio > 1:
            new_height = size
            new_width = int(ratio * new_height)
        else:
            new_width = size
            new_height = int(new_width / ratio)

        if max_size is not None and max(new_height, new_width) > max_size:
            # Need to recompute the aspect ratio, since it might have changed due to rounding
            ratio = new_width / new_height
            if ratio > 1:
                new_width = max_size
                new_height = int(new_width / ratio)
            else:
                new_height = max_size
                new_width = int(new_height * ratio)

        return new_height, new_width

    @pytest.mark.parametrize("size", OUTPUT_SIZES)
    @pytest.mark.parametrize("interpolation", INTERPOLATION_MODES)
    @pytest.mark.parametrize("use_max_size", [True, False])
    @pytest.mark.parametrize("antialias", [True, False])
    @pytest.mark.parametrize("dtype", [torch.float32, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_image(self, size, interpolation, use_max_size, antialias, dtype, device):
        if not (max_size_kwarg := self._make_max_size_kwarg(use_max_size=use_max_size, size=size)):
            return

        # In contrast to CPU, there is no native `InterpolationMode.BICUBIC` implementation for uint8 images on CUDA.
        # Internally, it uses the float path. Thus, we need to test with an enormous tolerance here to account for that.
        atol = 30 if transforms.InterpolationMode.BICUBIC and dtype is torch.uint8 else 1
        check_cuda_vs_cpu_tolerances = dict(rtol=0, atol=atol / 255 if dtype.is_floating_point else atol)

        check_kernel(
            F.resize_image,
            make_image(self.INPUT_SIZE, dtype=dtype, device=device),
            size=size,
            interpolation=interpolation,
            **max_size_kwarg,
            antialias=antialias,
            check_cuda_vs_cpu=check_cuda_vs_cpu_tolerances,
            check_scripted_vs_eager=not isinstance(size, int),
        )

    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("size", OUTPUT_SIZES)
    @pytest.mark.parametrize("use_max_size", [True, False])
    @pytest.mark.parametrize("dtype", [torch.float32, torch.int64])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_bounding_boxes(self, format, size, use_max_size, dtype, device):
        if not (max_size_kwarg := self._make_max_size_kwarg(use_max_size=use_max_size, size=size)):
            return

        bounding_boxes = make_bounding_boxes(
            format=format,
            canvas_size=self.INPUT_SIZE,
            dtype=dtype,
            device=device,
        )
        check_kernel(
            F.resize_bounding_boxes,
            bounding_boxes,
            canvas_size=bounding_boxes.canvas_size,
            size=size,
            **max_size_kwarg,
            check_scripted_vs_eager=not isinstance(size, int),
        )

    @pytest.mark.parametrize("make_mask", [make_segmentation_mask, make_detection_mask])
    def test_kernel_mask(self, make_mask):
        check_kernel(F.resize_mask, make_mask(self.INPUT_SIZE), size=self.OUTPUT_SIZES[-1])

    def test_kernel_video(self):
        check_kernel(F.resize_video, make_video(self.INPUT_SIZE), size=self.OUTPUT_SIZES[-1], antialias=True)

    @pytest.mark.parametrize("size", OUTPUT_SIZES)
    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    def test_functional(self, size, make_input):
        check_functional(
            F.resize,
            make_input(self.INPUT_SIZE),
            size=size,
            antialias=True,
            check_scripted_smoke=not isinstance(size, int),
        )

    @pytest.mark.parametrize(
        ("kernel", "input_type"),
        [
            (F.resize_image, torch.Tensor),
            (F._resize_image_pil, PIL.Image.Image),
            (F.resize_image, tv_tensors.Image),
            (F.resize_bounding_boxes, tv_tensors.BoundingBoxes),
            (F.resize_mask, tv_tensors.Mask),
            (F.resize_video, tv_tensors.Video),
        ],
    )
    def test_functional_signature(self, kernel, input_type):
        check_functional_kernel_signature_match(F.resize, kernel=kernel, input_type=input_type)

    @pytest.mark.parametrize("size", OUTPUT_SIZES)
    @pytest.mark.parametrize("device", cpu_and_cuda())
    @pytest.mark.parametrize(
        "make_input",
        [
            make_image_tensor,
            make_image_pil,
            make_image,
            make_bounding_boxes,
            make_segmentation_mask,
            make_detection_mask,
            make_video,
        ],
    )
    def test_transform(self, size, device, make_input):
        check_transform(
            transforms.Resize(size=size, antialias=True),
            make_input(self.INPUT_SIZE, device=device),
            # atol=1 due to Resize v2 is using native uint8 interpolate path for bilinear and nearest modes
            check_v1_compatibility=dict(rtol=0, atol=1),
        )

    def _check_output_size(self, input, output, *, size, max_size):
        assert tuple(F.get_size(output)) == self._compute_output_size(
            input_size=F.get_size(input), size=size, max_size=max_size
        )

    @pytest.mark.parametrize("size", OUTPUT_SIZES)
    # `InterpolationMode.NEAREST` is modeled after the buggy `INTER_NEAREST` interpolation of CV2.
    # The PIL equivalent of `InterpolationMode.NEAREST` is `InterpolationMode.NEAREST_EXACT`
    @pytest.mark.parametrize("interpolation", set(INTERPOLATION_MODES) - {transforms.InterpolationMode.NEAREST})
    @pytest.mark.parametrize("use_max_size", [True, False])
    @pytest.mark.parametrize("fn", [F.resize, transform_cls_to_functional(transforms.Resize)])
    def test_image_correctness(self, size, interpolation, use_max_size, fn):
        if not (max_size_kwarg := self._make_max_size_kwarg(use_max_size=use_max_size, size=size)):
            return

        image = make_image(self.INPUT_SIZE, dtype=torch.uint8)

        actual = fn(image, size=size, interpolation=interpolation, **max_size_kwarg, antialias=True)
        expected = F.to_image(F.resize(F.to_pil_image(image), size=size, interpolation=interpolation, **max_size_kwarg))

        self._check_output_size(image, actual, size=size, **max_size_kwarg)
        torch.testing.assert_close(actual, expected, atol=1, rtol=0)

    def _reference_resize_bounding_boxes(self, bounding_boxes, *, size, max_size=None):
        old_height, old_width = bounding_boxes.canvas_size
        new_height, new_width = self._compute_output_size(
            input_size=bounding_boxes.canvas_size, size=size, max_size=max_size
        )

        if (old_height, old_width) == (new_height, new_width):
            return bounding_boxes

        affine_matrix = np.array(
            [
                [new_width / old_width, 0, 0],
                [0, new_height / old_height, 0],
            ],
        )

        return reference_affine_bounding_boxes_helper(
            bounding_boxes,
            affine_matrix=affine_matrix,
            new_canvas_size=(new_height, new_width),
        )

    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("size", OUTPUT_SIZES)
    @pytest.mark.parametrize("use_max_size", [True, False])
    @pytest.mark.parametrize("fn", [F.resize, transform_cls_to_functional(transforms.Resize)])
    def test_bounding_boxes_correctness(self, format, size, use_max_size, fn):
        if not (max_size_kwarg := self._make_max_size_kwarg(use_max_size=use_max_size, size=size)):
            return

        bounding_boxes = make_bounding_boxes(format=format, canvas_size=self.INPUT_SIZE)

        actual = fn(bounding_boxes, size=size, **max_size_kwarg)
        expected = self._reference_resize_bounding_boxes(bounding_boxes, size=size, **max_size_kwarg)

        self._check_output_size(bounding_boxes, actual, size=size, **max_size_kwarg)
        torch.testing.assert_close(actual, expected)

    @pytest.mark.parametrize("interpolation", set(transforms.InterpolationMode) - set(INTERPOLATION_MODES))
    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_video],
    )
    def test_pil_interpolation_compat_smoke(self, interpolation, make_input):
        input = make_input(self.INPUT_SIZE)

        with (
            contextlib.nullcontext()
            if isinstance(input, PIL.Image.Image)
            # This error is triggered in PyTorch core
            else pytest.raises(NotImplementedError, match=f"got {interpolation.value.lower()}")
        ):
            F.resize(
                input,
                size=self.OUTPUT_SIZES[0],
                interpolation=interpolation,
            )

    def test_functional_pil_antialias_warning(self):
        with pytest.warns(UserWarning, match="Anti-alias option is always applied for PIL Image input"):
            F.resize(make_image_pil(self.INPUT_SIZE), size=self.OUTPUT_SIZES[0], antialias=False)

    @pytest.mark.parametrize("size", OUTPUT_SIZES)
    @pytest.mark.parametrize(
        "make_input",
        [
            make_image_tensor,
            make_image_pil,
            make_image,
            make_bounding_boxes,
            make_segmentation_mask,
            make_detection_mask,
            make_video,
        ],
    )
    def test_max_size_error(self, size, make_input):
        if isinstance(size, int) or len(size) == 1:
            max_size = (size if isinstance(size, int) else size[0]) - 1
            match = "must be strictly greater than the requested size"
        else:
            # value can be anything other than None
            max_size = -1
            match = "size should be an int or a sequence of length 1"

        with pytest.raises(ValueError, match=match):
            F.resize(make_input(self.INPUT_SIZE), size=size, max_size=max_size, antialias=True)

    @pytest.mark.parametrize("interpolation", INTERPOLATION_MODES)
    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image, make_video],
    )
    def test_antialias_warning(self, interpolation, make_input):
        with (
            assert_warns_antialias_default_value()
            if interpolation in {transforms.InterpolationMode.BILINEAR, transforms.InterpolationMode.BICUBIC}
            else assert_no_warnings()
        ):
            F.resize(
                make_input(self.INPUT_SIZE),
                size=self.OUTPUT_SIZES[0],
                interpolation=interpolation,
            )

    @pytest.mark.parametrize("interpolation", INTERPOLATION_MODES)
    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_video],
    )
    def test_interpolation_int(self, interpolation, make_input):
        input = make_input(self.INPUT_SIZE)

        # `InterpolationMode.NEAREST_EXACT` has no proper corresponding integer equivalent. Internally, we map it to
        # `0` to be the same as `InterpolationMode.NEAREST` for PIL. However, for the tensor backend there is a
        # difference and thus we don't test it here.
        if isinstance(input, torch.Tensor) and interpolation is transforms.InterpolationMode.NEAREST_EXACT:
            return

        expected = F.resize(input, size=self.OUTPUT_SIZES[0], interpolation=interpolation, antialias=True)
        actual = F.resize(
            input, size=self.OUTPUT_SIZES[0], interpolation=pil_modes_mapping[interpolation], antialias=True
        )

        assert_equal(actual, expected)

    def test_transform_unknown_size_error(self):
        with pytest.raises(ValueError, match="size can either be an integer or a list or tuple of one or two integers"):
            transforms.Resize(size=object())

    @pytest.mark.parametrize(
        "size", [min(INPUT_SIZE), [min(INPUT_SIZE)], (min(INPUT_SIZE),), list(INPUT_SIZE), tuple(INPUT_SIZE)]
    )
    @pytest.mark.parametrize(
        "make_input",
        [
            make_image_tensor,
            make_image_pil,
            make_image,
            make_bounding_boxes,
            make_segmentation_mask,
            make_detection_mask,
            make_video,
        ],
    )
    def test_noop(self, size, make_input):
        input = make_input(self.INPUT_SIZE)

        output = F.resize(input, size=F.get_size(input), antialias=True)

        # This identity check is not a requirement. It is here to avoid breaking the behavior by accident. If there
        # is a good reason to break this, feel free to downgrade to an equality check.
        if isinstance(input, tv_tensors.TVTensor):
            # We can't test identity directly, since that checks for the identity of the Python object. Since all
            # tv_tensors unwrap before a kernel and wrap again afterwards, the Python object changes. Thus, we check
            # that the underlying storage is the same
            assert output.data_ptr() == input.data_ptr()
        else:
            assert output is input

    @pytest.mark.parametrize(
        "make_input",
        [
            make_image_tensor,
            make_image_pil,
            make_image,
            make_bounding_boxes,
            make_segmentation_mask,
            make_detection_mask,
            make_video,
        ],
    )
    def test_no_regression_5405(self, make_input):
        # Checks that `max_size` is not ignored if `size == small_edge_size`
        # See https://github.com/pytorch/vision/issues/5405

        input = make_input(self.INPUT_SIZE)

        size = min(F.get_size(input))
        max_size = size + 1
        output = F.resize(input, size=size, max_size=max_size, antialias=True)

        assert max(F.get_size(output)) == max_size

    def _make_image(self, *args, batch_dims=(), memory_format=torch.contiguous_format, **kwargs):
        # torch.channels_last memory_format is only available for 4D tensors, i.e. (B, C, H, W). However, images coming
        # from PIL or our own I/O functions do not have a batch dimensions and are thus 3D, i.e. (C, H, W). Still, the
        # layout of the data in memory is channels last. To emulate this when a 3D input is requested here, we create
        # the image as 4D and create a view with the right shape afterwards. With this the layout in memory is channels
        # last although PyTorch doesn't recognizes it as such.
        emulate_channels_last = memory_format is torch.channels_last and len(batch_dims) != 1

        image = make_image(
            *args,
            batch_dims=(math.prod(batch_dims),) if emulate_channels_last else batch_dims,
            memory_format=memory_format,
            **kwargs,
        )

        if emulate_channels_last:
            image = tv_tensors.wrap(image.view(*batch_dims, *image.shape[-3:]), like=image)

        return image

    def _check_stride(self, image, *, memory_format):
        C, H, W = F.get_dimensions(image)
        if memory_format is torch.contiguous_format:
            expected_stride = (H * W, W, 1)
        elif memory_format is torch.channels_last:
            expected_stride = (1, W * C, C)
        else:
            raise ValueError(f"Unknown memory_format: {memory_format}")

        assert image.stride() == expected_stride

    # TODO: We can remove this test and related torchvision workaround
    #  once we fixed related pytorch issue: https://github.com/pytorch/pytorch/issues/68430
    @pytest.mark.parametrize("interpolation", INTERPOLATION_MODES)
    @pytest.mark.parametrize("antialias", [True, False])
    @pytest.mark.parametrize("memory_format", [torch.contiguous_format, torch.channels_last])
    @pytest.mark.parametrize("dtype", [torch.uint8, torch.float32])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_image_memory_format_consistency(self, interpolation, antialias, memory_format, dtype, device):
        size = self.OUTPUT_SIZES[0]

        input = self._make_image(self.INPUT_SIZE, dtype=dtype, device=device, memory_format=memory_format)

        # Smoke test to make sure we aren't starting with wrong assumptions
        self._check_stride(input, memory_format=memory_format)

        output = F.resize_image(input, size=size, interpolation=interpolation, antialias=antialias)

        self._check_stride(output, memory_format=memory_format)

    def test_float16_no_rounding(self):
        # Make sure Resize() doesn't round float16 images
        # Non-regression test for https://github.com/pytorch/vision/issues/7667

        input = make_image_tensor(self.INPUT_SIZE, dtype=torch.float16)
        output = F.resize_image(input, size=self.OUTPUT_SIZES[0], antialias=True)

        assert output.dtype is torch.float16
        assert (output.round() - output).abs().sum() > 0


class TestHorizontalFlip:
    @pytest.mark.parametrize("dtype", [torch.float32, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_image(self, dtype, device):
        check_kernel(F.horizontal_flip_image, make_image(dtype=dtype, device=device))

    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("dtype", [torch.float32, torch.int64])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_bounding_boxes(self, format, dtype, device):
        bounding_boxes = make_bounding_boxes(format=format, dtype=dtype, device=device)
        check_kernel(
            F.horizontal_flip_bounding_boxes,
            bounding_boxes,
            format=format,
            canvas_size=bounding_boxes.canvas_size,
        )

    @pytest.mark.parametrize("make_mask", [make_segmentation_mask, make_detection_mask])
    def test_kernel_mask(self, make_mask):
        check_kernel(F.horizontal_flip_mask, make_mask())

    def test_kernel_video(self):
        check_kernel(F.horizontal_flip_video, make_video())

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    def test_functional(self, make_input):
        check_functional(F.horizontal_flip, make_input())

    @pytest.mark.parametrize(
        ("kernel", "input_type"),
        [
            (F.horizontal_flip_image, torch.Tensor),
            (F._horizontal_flip_image_pil, PIL.Image.Image),
            (F.horizontal_flip_image, tv_tensors.Image),
            (F.horizontal_flip_bounding_boxes, tv_tensors.BoundingBoxes),
            (F.horizontal_flip_mask, tv_tensors.Mask),
            (F.horizontal_flip_video, tv_tensors.Video),
        ],
    )
    def test_functional_signature(self, kernel, input_type):
        check_functional_kernel_signature_match(F.horizontal_flip, kernel=kernel, input_type=input_type)

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_transform(self, make_input, device):
        check_transform(transforms.RandomHorizontalFlip(p=1), make_input(device=device))

    @pytest.mark.parametrize(
        "fn", [F.horizontal_flip, transform_cls_to_functional(transforms.RandomHorizontalFlip, p=1)]
    )
    def test_image_correctness(self, fn):
        image = make_image(dtype=torch.uint8, device="cpu")

        actual = fn(image)
        expected = F.to_image(F.horizontal_flip(F.to_pil_image(image)))

        torch.testing.assert_close(actual, expected)

    def _reference_horizontal_flip_bounding_boxes(self, bounding_boxes):
        affine_matrix = np.array(
            [
                [-1, 0, bounding_boxes.canvas_size[1]],
                [0, 1, 0],
            ],
        )

        return reference_affine_bounding_boxes_helper(bounding_boxes, affine_matrix=affine_matrix)

    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize(
        "fn", [F.horizontal_flip, transform_cls_to_functional(transforms.RandomHorizontalFlip, p=1)]
    )
    def test_bounding_boxes_correctness(self, format, fn):
        bounding_boxes = make_bounding_boxes(format=format)

        actual = fn(bounding_boxes)
        expected = self._reference_horizontal_flip_bounding_boxes(bounding_boxes)

        torch.testing.assert_close(actual, expected)

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_transform_noop(self, make_input, device):
        input = make_input(device=device)

        transform = transforms.RandomHorizontalFlip(p=0)

        output = transform(input)

        assert_equal(output, input)


class TestAffine:
    _EXHAUSTIVE_TYPE_AFFINE_KWARGS = dict(
        # float, int
        angle=[-10.9, 18],
        # two-list of float, two-list of int, two-tuple of float, two-tuple of int
        translate=[[6.3, -0.6], [1, -3], (16.6, -6.6), (-2, 4)],
        # float
        scale=[0.5],
        # float, int,
        # one-list of float, one-list of int, one-tuple of float, one-tuple of int
        # two-list of float, two-list of int, two-tuple of float, two-tuple of int
        shear=[35.6, 38, [-37.7], [-23], (5.3,), (-52,), [5.4, 21.8], [-47, 51], (-11.2, 36.7), (8, -53)],
        # None
        # two-list of float, two-list of int, two-tuple of float, two-tuple of int
        center=[None, [1.2, 4.9], [-3, 1], (2.5, -4.7), (3, 2)],
    )
    # The special case for shear makes sure we pick a value that is supported while JIT scripting
    _MINIMAL_AFFINE_KWARGS = {
        k: vs[0] if k != "shear" else next(v for v in vs if isinstance(v, list))
        for k, vs in _EXHAUSTIVE_TYPE_AFFINE_KWARGS.items()
    }
    _CORRECTNESS_AFFINE_KWARGS = {
        k: [v for v in vs if v is None or isinstance(v, float) or (isinstance(v, list) and len(v) > 1)]
        for k, vs in _EXHAUSTIVE_TYPE_AFFINE_KWARGS.items()
    }

    _EXHAUSTIVE_TYPE_TRANSFORM_AFFINE_RANGES = dict(
        degrees=[30, (-15, 20)],
        translate=[None, (0.5, 0.5)],
        scale=[None, (0.75, 1.25)],
        shear=[None, (12, 30, -17, 5), 10, (-5, 12)],
    )
    _CORRECTNESS_TRANSFORM_AFFINE_RANGES = {
        k: next(v for v in vs if v is not None) for k, vs in _EXHAUSTIVE_TYPE_TRANSFORM_AFFINE_RANGES.items()
    }

    def _check_kernel(self, kernel, input, *args, **kwargs):
        kwargs_ = self._MINIMAL_AFFINE_KWARGS.copy()
        kwargs_.update(kwargs)
        check_kernel(kernel, input, *args, **kwargs_)

    @param_value_parametrization(
        angle=_EXHAUSTIVE_TYPE_AFFINE_KWARGS["angle"],
        translate=_EXHAUSTIVE_TYPE_AFFINE_KWARGS["translate"],
        shear=_EXHAUSTIVE_TYPE_AFFINE_KWARGS["shear"],
        center=_EXHAUSTIVE_TYPE_AFFINE_KWARGS["center"],
        interpolation=[transforms.InterpolationMode.NEAREST, transforms.InterpolationMode.BILINEAR],
        fill=EXHAUSTIVE_TYPE_FILLS,
    )
    @pytest.mark.parametrize("dtype", [torch.float32, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_image(self, param, value, dtype, device):
        if param == "fill":
            value = adapt_fill(value, dtype=dtype)
        self._check_kernel(
            F.affine_image,
            make_image(dtype=dtype, device=device),
            **{param: value},
            check_scripted_vs_eager=not (param in {"shear", "fill"} and isinstance(value, (int, float))),
            check_cuda_vs_cpu=dict(atol=1, rtol=0)
            if dtype is torch.uint8 and param == "interpolation" and value is transforms.InterpolationMode.BILINEAR
            else True,
        )

    @param_value_parametrization(
        angle=_EXHAUSTIVE_TYPE_AFFINE_KWARGS["angle"],
        translate=_EXHAUSTIVE_TYPE_AFFINE_KWARGS["translate"],
        shear=_EXHAUSTIVE_TYPE_AFFINE_KWARGS["shear"],
        center=_EXHAUSTIVE_TYPE_AFFINE_KWARGS["center"],
    )
    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("dtype", [torch.float32, torch.int64])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_bounding_boxes(self, param, value, format, dtype, device):
        bounding_boxes = make_bounding_boxes(format=format, dtype=dtype, device=device)
        self._check_kernel(
            F.affine_bounding_boxes,
            bounding_boxes,
            format=format,
            canvas_size=bounding_boxes.canvas_size,
            **{param: value},
            check_scripted_vs_eager=not (param == "shear" and isinstance(value, (int, float))),
        )

    @pytest.mark.parametrize("make_mask", [make_segmentation_mask, make_detection_mask])
    def test_kernel_mask(self, make_mask):
        self._check_kernel(F.affine_mask, make_mask())

    def test_kernel_video(self):
        self._check_kernel(F.affine_video, make_video())

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    def test_functional(self, make_input):
        check_functional(F.affine, make_input(), **self._MINIMAL_AFFINE_KWARGS)

    @pytest.mark.parametrize(
        ("kernel", "input_type"),
        [
            (F.affine_image, torch.Tensor),
            (F._affine_image_pil, PIL.Image.Image),
            (F.affine_image, tv_tensors.Image),
            (F.affine_bounding_boxes, tv_tensors.BoundingBoxes),
            (F.affine_mask, tv_tensors.Mask),
            (F.affine_video, tv_tensors.Video),
        ],
    )
    def test_functional_signature(self, kernel, input_type):
        check_functional_kernel_signature_match(F.affine, kernel=kernel, input_type=input_type)

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_transform(self, make_input, device):
        input = make_input(device=device)

        check_transform(transforms.RandomAffine(**self._CORRECTNESS_TRANSFORM_AFFINE_RANGES), input)

    @pytest.mark.parametrize("angle", _CORRECTNESS_AFFINE_KWARGS["angle"])
    @pytest.mark.parametrize("translate", _CORRECTNESS_AFFINE_KWARGS["translate"])
    @pytest.mark.parametrize("scale", _CORRECTNESS_AFFINE_KWARGS["scale"])
    @pytest.mark.parametrize("shear", _CORRECTNESS_AFFINE_KWARGS["shear"])
    @pytest.mark.parametrize("center", _CORRECTNESS_AFFINE_KWARGS["center"])
    @pytest.mark.parametrize(
        "interpolation", [transforms.InterpolationMode.NEAREST, transforms.InterpolationMode.BILINEAR]
    )
    @pytest.mark.parametrize("fill", CORRECTNESS_FILLS)
    def test_functional_image_correctness(self, angle, translate, scale, shear, center, interpolation, fill):
        image = make_image(dtype=torch.uint8, device="cpu")

        fill = adapt_fill(fill, dtype=torch.uint8)

        actual = F.affine(
            image,
            angle=angle,
            translate=translate,
            scale=scale,
            shear=shear,
            center=center,
            interpolation=interpolation,
            fill=fill,
        )
        expected = F.to_image(
            F.affine(
                F.to_pil_image(image),
                angle=angle,
                translate=translate,
                scale=scale,
                shear=shear,
                center=center,
                interpolation=interpolation,
                fill=fill,
            )
        )

        mae = (actual.float() - expected.float()).abs().mean()
        assert mae < 2 if interpolation is transforms.InterpolationMode.NEAREST else 8

    @pytest.mark.parametrize("center", _CORRECTNESS_AFFINE_KWARGS["center"])
    @pytest.mark.parametrize(
        "interpolation", [transforms.InterpolationMode.NEAREST, transforms.InterpolationMode.BILINEAR]
    )
    @pytest.mark.parametrize("fill", CORRECTNESS_FILLS)
    @pytest.mark.parametrize("seed", list(range(5)))
    def test_transform_image_correctness(self, center, interpolation, fill, seed):
        image = make_image(dtype=torch.uint8, device="cpu")

        fill = adapt_fill(fill, dtype=torch.uint8)

        transform = transforms.RandomAffine(
            **self._CORRECTNESS_TRANSFORM_AFFINE_RANGES, center=center, interpolation=interpolation, fill=fill
        )

        torch.manual_seed(seed)
        actual = transform(image)

        torch.manual_seed(seed)
        expected = F.to_image(transform(F.to_pil_image(image)))

        mae = (actual.float() - expected.float()).abs().mean()
        assert mae < 2 if interpolation is transforms.InterpolationMode.NEAREST else 8

    def _compute_affine_matrix(self, *, angle, translate, scale, shear, center):
        rot = math.radians(angle)
        cx, cy = center
        tx, ty = translate
        sx, sy = [math.radians(s) for s in ([shear, 0.0] if isinstance(shear, (int, float)) else shear)]

        c_matrix = np.array([[1, 0, cx], [0, 1, cy], [0, 0, 1]])
        t_matrix = np.array([[1, 0, tx], [0, 1, ty], [0, 0, 1]])
        c_matrix_inv = np.linalg.inv(c_matrix)
        rs_matrix = np.array(
            [
                [scale * math.cos(rot), -scale * math.sin(rot), 0],
                [scale * math.sin(rot), scale * math.cos(rot), 0],
                [0, 0, 1],
            ]
        )
        shear_x_matrix = np.array([[1, -math.tan(sx), 0], [0, 1, 0], [0, 0, 1]])
        shear_y_matrix = np.array([[1, 0, 0], [-math.tan(sy), 1, 0], [0, 0, 1]])
        rss_matrix = np.matmul(rs_matrix, np.matmul(shear_y_matrix, shear_x_matrix))
        true_matrix = np.matmul(t_matrix, np.matmul(c_matrix, np.matmul(rss_matrix, c_matrix_inv)))
        return true_matrix[:2, :]

    def _reference_affine_bounding_boxes(self, bounding_boxes, *, angle, translate, scale, shear, center):
        if center is None:
            center = [s * 0.5 for s in bounding_boxes.canvas_size[::-1]]

        return reference_affine_bounding_boxes_helper(
            bounding_boxes,
            affine_matrix=self._compute_affine_matrix(
                angle=angle, translate=translate, scale=scale, shear=shear, center=center
            ),
        )

    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("angle", _CORRECTNESS_AFFINE_KWARGS["angle"])
    @pytest.mark.parametrize("translate", _CORRECTNESS_AFFINE_KWARGS["translate"])
    @pytest.mark.parametrize("scale", _CORRECTNESS_AFFINE_KWARGS["scale"])
    @pytest.mark.parametrize("shear", _CORRECTNESS_AFFINE_KWARGS["shear"])
    @pytest.mark.parametrize("center", _CORRECTNESS_AFFINE_KWARGS["center"])
    def test_functional_bounding_boxes_correctness(self, format, angle, translate, scale, shear, center):
        bounding_boxes = make_bounding_boxes(format=format)

        actual = F.affine(
            bounding_boxes,
            angle=angle,
            translate=translate,
            scale=scale,
            shear=shear,
            center=center,
        )
        expected = self._reference_affine_bounding_boxes(
            bounding_boxes,
            angle=angle,
            translate=translate,
            scale=scale,
            shear=shear,
            center=center,
        )

        torch.testing.assert_close(actual, expected)

    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("center", _CORRECTNESS_AFFINE_KWARGS["center"])
    @pytest.mark.parametrize("seed", list(range(5)))
    def test_transform_bounding_boxes_correctness(self, format, center, seed):
        bounding_boxes = make_bounding_boxes(format=format)

        transform = transforms.RandomAffine(**self._CORRECTNESS_TRANSFORM_AFFINE_RANGES, center=center)

        torch.manual_seed(seed)
        params = transform._get_params([bounding_boxes])

        torch.manual_seed(seed)
        actual = transform(bounding_boxes)

        expected = self._reference_affine_bounding_boxes(bounding_boxes, **params, center=center)

        torch.testing.assert_close(actual, expected)

    @pytest.mark.parametrize("degrees", _EXHAUSTIVE_TYPE_TRANSFORM_AFFINE_RANGES["degrees"])
    @pytest.mark.parametrize("translate", _EXHAUSTIVE_TYPE_TRANSFORM_AFFINE_RANGES["translate"])
    @pytest.mark.parametrize("scale", _EXHAUSTIVE_TYPE_TRANSFORM_AFFINE_RANGES["scale"])
    @pytest.mark.parametrize("shear", _EXHAUSTIVE_TYPE_TRANSFORM_AFFINE_RANGES["shear"])
    @pytest.mark.parametrize("seed", list(range(10)))
    def test_transform_get_params_bounds(self, degrees, translate, scale, shear, seed):
        image = make_image()
        height, width = F.get_size(image)

        transform = transforms.RandomAffine(degrees=degrees, translate=translate, scale=scale, shear=shear)

        torch.manual_seed(seed)
        params = transform._get_params([image])

        if isinstance(degrees, (int, float)):
            assert -degrees <= params["angle"] <= degrees
        else:
            assert degrees[0] <= params["angle"] <= degrees[1]

        if translate is not None:
            width_max = int(round(translate[0] * width))
            height_max = int(round(translate[1] * height))
            assert -width_max <= params["translate"][0] <= width_max
            assert -height_max <= params["translate"][1] <= height_max
        else:
            assert params["translate"] == (0, 0)

        if scale is not None:
            assert scale[0] <= params["scale"] <= scale[1]
        else:
            assert params["scale"] == 1.0

        if shear is not None:
            if isinstance(shear, (int, float)):
                assert -shear <= params["shear"][0] <= shear
                assert params["shear"][1] == 0.0
            elif len(shear) == 2:
                assert shear[0] <= params["shear"][0] <= shear[1]
                assert params["shear"][1] == 0.0
            elif len(shear) == 4:
                assert shear[0] <= params["shear"][0] <= shear[1]
                assert shear[2] <= params["shear"][1] <= shear[3]
        else:
            assert params["shear"] == (0, 0)

    @pytest.mark.parametrize("param", ["degrees", "translate", "scale", "shear", "center"])
    @pytest.mark.parametrize("value", [0, [0], [0, 0, 0]])
    def test_transform_sequence_len_errors(self, param, value):
        if param in {"degrees", "shear"} and not isinstance(value, list):
            return

        kwargs = {param: value}
        if param != "degrees":
            kwargs["degrees"] = 0

        with pytest.raises(
            ValueError if isinstance(value, list) else TypeError, match=f"{param} should be a sequence of length 2"
        ):
            transforms.RandomAffine(**kwargs)

    def test_transform_negative_degrees_error(self):
        with pytest.raises(ValueError, match="If degrees is a single number, it must be positive"):
            transforms.RandomAffine(degrees=-1)

    @pytest.mark.parametrize("translate", [[-1, 0], [2, 0], [-1, 2]])
    def test_transform_translate_range_error(self, translate):
        with pytest.raises(ValueError, match="translation values should be between 0 and 1"):
            transforms.RandomAffine(degrees=0, translate=translate)

    @pytest.mark.parametrize("scale", [[-1, 0], [0, -1], [-1, -1]])
    def test_transform_scale_range_error(self, scale):
        with pytest.raises(ValueError, match="scale values should be positive"):
            transforms.RandomAffine(degrees=0, scale=scale)

    def test_transform_negative_shear_error(self):
        with pytest.raises(ValueError, match="If shear is a single number, it must be positive"):
            transforms.RandomAffine(degrees=0, shear=-1)

    def test_transform_unknown_fill_error(self):
        with pytest.raises(TypeError, match="Got inappropriate fill arg"):
            transforms.RandomAffine(degrees=0, fill="fill")


class TestVerticalFlip:
    @pytest.mark.parametrize("dtype", [torch.float32, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_image(self, dtype, device):
        check_kernel(F.vertical_flip_image, make_image(dtype=dtype, device=device))

    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("dtype", [torch.float32, torch.int64])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_bounding_boxes(self, format, dtype, device):
        bounding_boxes = make_bounding_boxes(format=format, dtype=dtype, device=device)
        check_kernel(
            F.vertical_flip_bounding_boxes,
            bounding_boxes,
            format=format,
            canvas_size=bounding_boxes.canvas_size,
        )

    @pytest.mark.parametrize("make_mask", [make_segmentation_mask, make_detection_mask])
    def test_kernel_mask(self, make_mask):
        check_kernel(F.vertical_flip_mask, make_mask())

    def test_kernel_video(self):
        check_kernel(F.vertical_flip_video, make_video())

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    def test_functional(self, make_input):
        check_functional(F.vertical_flip, make_input())

    @pytest.mark.parametrize(
        ("kernel", "input_type"),
        [
            (F.vertical_flip_image, torch.Tensor),
            (F._vertical_flip_image_pil, PIL.Image.Image),
            (F.vertical_flip_image, tv_tensors.Image),
            (F.vertical_flip_bounding_boxes, tv_tensors.BoundingBoxes),
            (F.vertical_flip_mask, tv_tensors.Mask),
            (F.vertical_flip_video, tv_tensors.Video),
        ],
    )
    def test_functional_signature(self, kernel, input_type):
        check_functional_kernel_signature_match(F.vertical_flip, kernel=kernel, input_type=input_type)

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_transform(self, make_input, device):
        check_transform(transforms.RandomVerticalFlip(p=1), make_input(device=device))

    @pytest.mark.parametrize("fn", [F.vertical_flip, transform_cls_to_functional(transforms.RandomVerticalFlip, p=1)])
    def test_image_correctness(self, fn):
        image = make_image(dtype=torch.uint8, device="cpu")

        actual = fn(image)
        expected = F.to_image(F.vertical_flip(F.to_pil_image(image)))

        torch.testing.assert_close(actual, expected)

    def _reference_vertical_flip_bounding_boxes(self, bounding_boxes):
        affine_matrix = np.array(
            [
                [1, 0, 0],
                [0, -1, bounding_boxes.canvas_size[0]],
            ],
        )

        return reference_affine_bounding_boxes_helper(bounding_boxes, affine_matrix=affine_matrix)

    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("fn", [F.vertical_flip, transform_cls_to_functional(transforms.RandomVerticalFlip, p=1)])
    def test_bounding_boxes_correctness(self, format, fn):
        bounding_boxes = make_bounding_boxes(format=format)

        actual = fn(bounding_boxes)
        expected = self._reference_vertical_flip_bounding_boxes(bounding_boxes)

        torch.testing.assert_close(actual, expected)

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_transform_noop(self, make_input, device):
        input = make_input(device=device)

        transform = transforms.RandomVerticalFlip(p=0)

        output = transform(input)

        assert_equal(output, input)


class TestRotate:
    _EXHAUSTIVE_TYPE_AFFINE_KWARGS = dict(
        # float, int
        angle=[-10.9, 18],
        # None
        # two-list of float, two-list of int, two-tuple of float, two-tuple of int
        center=[None, [1.2, 4.9], [-3, 1], (2.5, -4.7), (3, 2)],
    )
    _MINIMAL_AFFINE_KWARGS = {k: vs[0] for k, vs in _EXHAUSTIVE_TYPE_AFFINE_KWARGS.items()}
    _CORRECTNESS_AFFINE_KWARGS = {
        k: [v for v in vs if v is None or isinstance(v, float) or isinstance(v, list)]
        for k, vs in _EXHAUSTIVE_TYPE_AFFINE_KWARGS.items()
    }

    _EXHAUSTIVE_TYPE_TRANSFORM_AFFINE_RANGES = dict(
        degrees=[30, (-15, 20)],
    )
    _CORRECTNESS_TRANSFORM_AFFINE_RANGES = {k: vs[0] for k, vs in _EXHAUSTIVE_TYPE_TRANSFORM_AFFINE_RANGES.items()}

    @param_value_parametrization(
        angle=_EXHAUSTIVE_TYPE_AFFINE_KWARGS["angle"],
        interpolation=[transforms.InterpolationMode.NEAREST, transforms.InterpolationMode.BILINEAR],
        expand=[False, True],
        center=_EXHAUSTIVE_TYPE_AFFINE_KWARGS["center"],
        fill=EXHAUSTIVE_TYPE_FILLS,
    )
    @pytest.mark.parametrize("dtype", [torch.float32, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_image(self, param, value, dtype, device):
        kwargs = {param: value}
        if param != "angle":
            kwargs["angle"] = self._MINIMAL_AFFINE_KWARGS["angle"]
        check_kernel(
            F.rotate_image,
            make_image(dtype=dtype, device=device),
            **kwargs,
            check_scripted_vs_eager=not (param == "fill" and isinstance(value, (int, float))),
        )

    @param_value_parametrization(
        angle=_EXHAUSTIVE_TYPE_AFFINE_KWARGS["angle"],
        expand=[False, True],
        center=_EXHAUSTIVE_TYPE_AFFINE_KWARGS["center"],
    )
    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("dtype", [torch.float32, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_bounding_boxes(self, param, value, format, dtype, device):
        kwargs = {param: value}
        if param != "angle":
            kwargs["angle"] = self._MINIMAL_AFFINE_KWARGS["angle"]

        bounding_boxes = make_bounding_boxes(format=format, dtype=dtype, device=device)

        check_kernel(
            F.rotate_bounding_boxes,
            bounding_boxes,
            format=format,
            canvas_size=bounding_boxes.canvas_size,
            **kwargs,
        )

    @pytest.mark.parametrize("make_mask", [make_segmentation_mask, make_detection_mask])
    def test_kernel_mask(self, make_mask):
        check_kernel(F.rotate_mask, make_mask(), **self._MINIMAL_AFFINE_KWARGS)

    def test_kernel_video(self):
        check_kernel(F.rotate_video, make_video(), **self._MINIMAL_AFFINE_KWARGS)

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    def test_functional(self, make_input):
        check_functional(F.rotate, make_input(), **self._MINIMAL_AFFINE_KWARGS)

    @pytest.mark.parametrize(
        ("kernel", "input_type"),
        [
            (F.rotate_image, torch.Tensor),
            (F._rotate_image_pil, PIL.Image.Image),
            (F.rotate_image, tv_tensors.Image),
            (F.rotate_bounding_boxes, tv_tensors.BoundingBoxes),
            (F.rotate_mask, tv_tensors.Mask),
            (F.rotate_video, tv_tensors.Video),
        ],
    )
    def test_functional_signature(self, kernel, input_type):
        check_functional_kernel_signature_match(F.rotate, kernel=kernel, input_type=input_type)

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_transform(self, make_input, device):
        check_transform(
            transforms.RandomRotation(**self._CORRECTNESS_TRANSFORM_AFFINE_RANGES), make_input(device=device)
        )

    @pytest.mark.parametrize("angle", _CORRECTNESS_AFFINE_KWARGS["angle"])
    @pytest.mark.parametrize("center", _CORRECTNESS_AFFINE_KWARGS["center"])
    @pytest.mark.parametrize(
        "interpolation", [transforms.InterpolationMode.NEAREST, transforms.InterpolationMode.BILINEAR]
    )
    @pytest.mark.parametrize("expand", [False, True])
    @pytest.mark.parametrize("fill", CORRECTNESS_FILLS)
    def test_functional_image_correctness(self, angle, center, interpolation, expand, fill):
        image = make_image(dtype=torch.uint8, device="cpu")

        fill = adapt_fill(fill, dtype=torch.uint8)

        actual = F.rotate(image, angle=angle, center=center, interpolation=interpolation, expand=expand, fill=fill)
        expected = F.to_image(
            F.rotate(
                F.to_pil_image(image), angle=angle, center=center, interpolation=interpolation, expand=expand, fill=fill
            )
        )

        mae = (actual.float() - expected.float()).abs().mean()
        assert mae < 1 if interpolation is transforms.InterpolationMode.NEAREST else 6

    @pytest.mark.parametrize("center", _CORRECTNESS_AFFINE_KWARGS["center"])
    @pytest.mark.parametrize(
        "interpolation", [transforms.InterpolationMode.NEAREST, transforms.InterpolationMode.BILINEAR]
    )
    @pytest.mark.parametrize("expand", [False, True])
    @pytest.mark.parametrize("fill", CORRECTNESS_FILLS)
    @pytest.mark.parametrize("seed", list(range(5)))
    def test_transform_image_correctness(self, center, interpolation, expand, fill, seed):
        image = make_image(dtype=torch.uint8, device="cpu")

        fill = adapt_fill(fill, dtype=torch.uint8)

        transform = transforms.RandomRotation(
            **self._CORRECTNESS_TRANSFORM_AFFINE_RANGES,
            center=center,
            interpolation=interpolation,
            expand=expand,
            fill=fill,
        )

        torch.manual_seed(seed)
        actual = transform(image)

        torch.manual_seed(seed)
        expected = F.to_image(transform(F.to_pil_image(image)))

        mae = (actual.float() - expected.float()).abs().mean()
        assert mae < 1 if interpolation is transforms.InterpolationMode.NEAREST else 6

    def _compute_output_canvas_size(self, *, expand, canvas_size, affine_matrix):
        if not expand:
            return canvas_size, (0.0, 0.0)

        input_height, input_width = canvas_size

        input_image_frame = np.array(
            [
                [0.0, 0.0, 1.0],
                [0.0, input_height, 1.0],
                [input_width, input_height, 1.0],
                [input_width, 0.0, 1.0],
            ],
            dtype=np.float64,
        )
        output_image_frame = np.matmul(input_image_frame, affine_matrix.astype(input_image_frame.dtype).T)

        recenter_x = float(np.min(output_image_frame[:, 0]))
        recenter_y = float(np.min(output_image_frame[:, 1]))

        output_width = int(np.max(output_image_frame[:, 0]) - recenter_x)
        output_height = int(np.max(output_image_frame[:, 1]) - recenter_y)

        return (output_height, output_width), (recenter_x, recenter_y)

    def _recenter_bounding_boxes_after_expand(self, bounding_boxes, *, recenter_xy):
        x, y = recenter_xy
        if bounding_boxes.format is tv_tensors.BoundingBoxFormat.XYXY:
            translate = [x, y, x, y]
        else:
            translate = [x, y, 0.0, 0.0]
        return tv_tensors.wrap(
            (bounding_boxes.to(torch.float64) - torch.tensor(translate)).to(bounding_boxes.dtype), like=bounding_boxes
        )

    def _reference_rotate_bounding_boxes(self, bounding_boxes, *, angle, expand, center):
        if center is None:
            center = [s * 0.5 for s in bounding_boxes.canvas_size[::-1]]
        cx, cy = center

        a = np.cos(angle * np.pi / 180.0)
        b = np.sin(angle * np.pi / 180.0)
        affine_matrix = np.array(
            [
                [a, b, cx - cx * a - b * cy],
                [-b, a, cy + cx * b - a * cy],
            ],
        )

        new_canvas_size, recenter_xy = self._compute_output_canvas_size(
            expand=expand, canvas_size=bounding_boxes.canvas_size, affine_matrix=affine_matrix
        )

        output = reference_affine_bounding_boxes_helper(
            bounding_boxes,
            affine_matrix=affine_matrix,
            new_canvas_size=new_canvas_size,
            clamp=False,
        )

        return F.clamp_bounding_boxes(self._recenter_bounding_boxes_after_expand(output, recenter_xy=recenter_xy)).to(
            bounding_boxes
        )

    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("angle", _CORRECTNESS_AFFINE_KWARGS["angle"])
    @pytest.mark.parametrize("expand", [False, True])
    @pytest.mark.parametrize("center", _CORRECTNESS_AFFINE_KWARGS["center"])
    def test_functional_bounding_boxes_correctness(self, format, angle, expand, center):
        bounding_boxes = make_bounding_boxes(format=format)

        actual = F.rotate(bounding_boxes, angle=angle, expand=expand, center=center)
        expected = self._reference_rotate_bounding_boxes(bounding_boxes, angle=angle, expand=expand, center=center)

        torch.testing.assert_close(actual, expected)
        torch.testing.assert_close(F.get_size(actual), F.get_size(expected), atol=2 if expand else 0, rtol=0)

    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("expand", [False, True])
    @pytest.mark.parametrize("center", _CORRECTNESS_AFFINE_KWARGS["center"])
    @pytest.mark.parametrize("seed", list(range(5)))
    def test_transform_bounding_boxes_correctness(self, format, expand, center, seed):
        bounding_boxes = make_bounding_boxes(format=format)

        transform = transforms.RandomRotation(**self._CORRECTNESS_TRANSFORM_AFFINE_RANGES, expand=expand, center=center)

        torch.manual_seed(seed)
        params = transform._get_params([bounding_boxes])

        torch.manual_seed(seed)
        actual = transform(bounding_boxes)

        expected = self._reference_rotate_bounding_boxes(bounding_boxes, **params, expand=expand, center=center)

        torch.testing.assert_close(actual, expected)
        torch.testing.assert_close(F.get_size(actual), F.get_size(expected), atol=2 if expand else 0, rtol=0)

    @pytest.mark.parametrize("degrees", _EXHAUSTIVE_TYPE_TRANSFORM_AFFINE_RANGES["degrees"])
    @pytest.mark.parametrize("seed", list(range(10)))
    def test_transform_get_params_bounds(self, degrees, seed):
        transform = transforms.RandomRotation(degrees=degrees)

        torch.manual_seed(seed)
        params = transform._get_params([])

        if isinstance(degrees, (int, float)):
            assert -degrees <= params["angle"] <= degrees
        else:
            assert degrees[0] <= params["angle"] <= degrees[1]

    @pytest.mark.parametrize("param", ["degrees", "center"])
    @pytest.mark.parametrize("value", [0, [0], [0, 0, 0]])
    def test_transform_sequence_len_errors(self, param, value):
        if param == "degrees" and not isinstance(value, list):
            return

        kwargs = {param: value}
        if param != "degrees":
            kwargs["degrees"] = 0

        with pytest.raises(
            ValueError if isinstance(value, list) else TypeError, match=f"{param} should be a sequence of length 2"
        ):
            transforms.RandomRotation(**kwargs)

    def test_transform_negative_degrees_error(self):
        with pytest.raises(ValueError, match="If degrees is a single number, it must be positive"):
            transforms.RandomAffine(degrees=-1)

    def test_transform_unknown_fill_error(self):
        with pytest.raises(TypeError, match="Got inappropriate fill arg"):
            transforms.RandomAffine(degrees=0, fill="fill")


class TestCompose:
    class BuiltinTransform(transforms.Transform):
        def _transform(self, inpt, params):
            return inpt

    class PackedInputTransform(nn.Module):
        def forward(self, sample):
            assert len(sample) == 2
            return sample

    class UnpackedInputTransform(nn.Module):
        def forward(self, image, label):
            return image, label

    @pytest.mark.parametrize(
        "transform_clss",
        [
            [BuiltinTransform],
            [PackedInputTransform],
            [UnpackedInputTransform],
            [BuiltinTransform, BuiltinTransform],
            [PackedInputTransform, PackedInputTransform],
            [UnpackedInputTransform, UnpackedInputTransform],
            [BuiltinTransform, PackedInputTransform, BuiltinTransform],
            [BuiltinTransform, UnpackedInputTransform, BuiltinTransform],
            [PackedInputTransform, BuiltinTransform, PackedInputTransform],
            [UnpackedInputTransform, BuiltinTransform, UnpackedInputTransform],
        ],
    )
    @pytest.mark.parametrize("unpack", [True, False])
    def test_packed_unpacked(self, transform_clss, unpack):
        needs_packed_inputs = any(issubclass(cls, self.PackedInputTransform) for cls in transform_clss)
        needs_unpacked_inputs = any(issubclass(cls, self.UnpackedInputTransform) for cls in transform_clss)
        assert not (needs_packed_inputs and needs_unpacked_inputs)

        transform = transforms.Compose([cls() for cls in transform_clss])

        image = make_image()
        label = 3
        packed_input = (image, label)

        def call_transform():
            if unpack:
                return transform(*packed_input)
            else:
                return transform(packed_input)

        if needs_unpacked_inputs and not unpack:
            with pytest.raises(TypeError, match="missing 1 required positional argument"):
                call_transform()
        elif needs_packed_inputs and unpack:
            with pytest.raises(TypeError, match="takes 2 positional arguments but 3 were given"):
                call_transform()
        else:
            output = call_transform()

            assert isinstance(output, tuple) and len(output) == 2
            assert output[0] is image
            assert output[1] is label


class TestToDtype:
    @pytest.mark.parametrize(
        ("kernel", "make_input"),
        [
            (F.to_dtype_image, make_image_tensor),
            (F.to_dtype_image, make_image),
            (F.to_dtype_video, make_video),
        ],
    )
    @pytest.mark.parametrize("input_dtype", [torch.float32, torch.float64, torch.uint8])
    @pytest.mark.parametrize("output_dtype", [torch.float32, torch.float64, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    @pytest.mark.parametrize("scale", (True, False))
    def test_kernel(self, kernel, make_input, input_dtype, output_dtype, device, scale):
        check_kernel(
            kernel,
            make_input(dtype=input_dtype, device=device),
            expect_same_dtype=input_dtype is output_dtype,
            dtype=output_dtype,
            scale=scale,
        )

    @pytest.mark.parametrize("make_input", [make_image_tensor, make_image, make_video])
    @pytest.mark.parametrize("input_dtype", [torch.float32, torch.float64, torch.uint8])
    @pytest.mark.parametrize("output_dtype", [torch.float32, torch.float64, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    @pytest.mark.parametrize("scale", (True, False))
    def test_functional(self, make_input, input_dtype, output_dtype, device, scale):
        check_functional(
            F.to_dtype,
            make_input(dtype=input_dtype, device=device),
            dtype=output_dtype,
            scale=scale,
        )

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    @pytest.mark.parametrize("input_dtype", [torch.float32, torch.float64, torch.uint8])
    @pytest.mark.parametrize("output_dtype", [torch.float32, torch.float64, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    @pytest.mark.parametrize("scale", (True, False))
    @pytest.mark.parametrize("as_dict", (True, False))
    def test_transform(self, make_input, input_dtype, output_dtype, device, scale, as_dict):
        input = make_input(dtype=input_dtype, device=device)
        if as_dict:
            output_dtype = {type(input): output_dtype}
        check_transform(transforms.ToDtype(dtype=output_dtype, scale=scale), input)

    def reference_convert_dtype_image_tensor(self, image, dtype=torch.float, scale=False):
        input_dtype = image.dtype
        output_dtype = dtype

        if not scale:
            return image.to(dtype)

        if output_dtype == input_dtype:
            return image

        def fn(value):
            if input_dtype.is_floating_point:
                if output_dtype.is_floating_point:
                    return value
                else:
                    return round(decimal.Decimal(value) * torch.iinfo(output_dtype).max)
            else:
                input_max_value = torch.iinfo(input_dtype).max

                if output_dtype.is_floating_point:
                    return float(decimal.Decimal(value) / input_max_value)
                else:
                    output_max_value = torch.iinfo(output_dtype).max

                    if input_max_value > output_max_value:
                        factor = (input_max_value + 1) // (output_max_value + 1)
                        return value / factor
                    else:
                        factor = (output_max_value + 1) // (input_max_value + 1)
                        return value * factor

        return torch.tensor(tree_map(fn, image.tolist()), dtype=dtype, device=image.device)

    @pytest.mark.parametrize("input_dtype", [torch.float32, torch.float64, torch.uint8])
    @pytest.mark.parametrize("output_dtype", [torch.float32, torch.float64, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    @pytest.mark.parametrize("scale", (True, False))
    def test_image_correctness(self, input_dtype, output_dtype, device, scale):
        if input_dtype.is_floating_point and output_dtype == torch.int64:
            pytest.xfail("float to int64 conversion is not supported")

        input = make_image(dtype=input_dtype, device=device)

        out = F.to_dtype(input, dtype=output_dtype, scale=scale)
        expected = self.reference_convert_dtype_image_tensor(input, dtype=output_dtype, scale=scale)

        if input_dtype.is_floating_point and not output_dtype.is_floating_point and scale:
            torch.testing.assert_close(out, expected, atol=1, rtol=0)
        else:
            torch.testing.assert_close(out, expected)

    def was_scaled(self, inpt):
        # this assumes the target dtype is float
        return inpt.max() <= 1

    def make_inpt_with_bbox_and_mask(self, make_input):
        H, W = 10, 10
        inpt_dtype = torch.uint8
        bbox_dtype = torch.float32
        mask_dtype = torch.bool
        sample = {
            "inpt": make_input(size=(H, W), dtype=inpt_dtype),
            "bbox": make_bounding_boxes(canvas_size=(H, W), dtype=bbox_dtype),
            "mask": make_detection_mask(size=(H, W), dtype=mask_dtype),
        }

        return sample, inpt_dtype, bbox_dtype, mask_dtype

    @pytest.mark.parametrize("make_input", (make_image_tensor, make_image, make_video))
    @pytest.mark.parametrize("scale", (True, False))
    def test_dtype_not_a_dict(self, make_input, scale):
        # assert only inpt gets transformed when dtype isn't a dict

        sample, inpt_dtype, bbox_dtype, mask_dtype = self.make_inpt_with_bbox_and_mask(make_input)
        out = transforms.ToDtype(dtype=torch.float32, scale=scale)(sample)

        assert out["inpt"].dtype != inpt_dtype
        assert out["inpt"].dtype == torch.float32
        if scale:
            assert self.was_scaled(out["inpt"])
        else:
            assert not self.was_scaled(out["inpt"])
        assert out["bbox"].dtype == bbox_dtype
        assert out["mask"].dtype == mask_dtype

    @pytest.mark.parametrize("make_input", (make_image_tensor, make_image, make_video))
    def test_others_catch_all_and_none(self, make_input):
        # make sure "others" works as a catch-all and that None means no conversion

        sample, inpt_dtype, bbox_dtype, mask_dtype = self.make_inpt_with_bbox_and_mask(make_input)
        out = transforms.ToDtype(dtype={tv_tensors.Mask: torch.int64, "others": None})(sample)
        assert out["inpt"].dtype == inpt_dtype
        assert out["bbox"].dtype == bbox_dtype
        assert out["mask"].dtype != mask_dtype
        assert out["mask"].dtype == torch.int64

    @pytest.mark.parametrize("make_input", (make_image_tensor, make_image, make_video))
    def test_typical_use_case(self, make_input):
        # Typical use-case: want to convert dtype and scale for inpt and just dtype for masks.
        # This just makes sure we now have a decent API for this

        sample, inpt_dtype, bbox_dtype, mask_dtype = self.make_inpt_with_bbox_and_mask(make_input)
        out = transforms.ToDtype(
            dtype={type(sample["inpt"]): torch.float32, tv_tensors.Mask: torch.int64, "others": None}, scale=True
        )(sample)
        assert out["inpt"].dtype != inpt_dtype
        assert out["inpt"].dtype == torch.float32
        assert self.was_scaled(out["inpt"])
        assert out["bbox"].dtype == bbox_dtype
        assert out["mask"].dtype != mask_dtype
        assert out["mask"].dtype == torch.int64

    @pytest.mark.parametrize("make_input", (make_image_tensor, make_image, make_video))
    def test_errors_warnings(self, make_input):
        sample, inpt_dtype, bbox_dtype, mask_dtype = self.make_inpt_with_bbox_and_mask(make_input)

        with pytest.raises(ValueError, match="No dtype was specified for"):
            out = transforms.ToDtype(dtype={tv_tensors.Mask: torch.float32})(sample)
        with pytest.warns(UserWarning, match=re.escape("plain `torch.Tensor` will *not* be transformed")):
            transforms.ToDtype(dtype={torch.Tensor: torch.float32, tv_tensors.Image: torch.float32})
        with pytest.warns(UserWarning, match="no scaling will be done"):
            out = transforms.ToDtype(dtype={"others": None}, scale=True)(sample)
        assert out["inpt"].dtype == inpt_dtype
        assert out["bbox"].dtype == bbox_dtype
        assert out["mask"].dtype == mask_dtype


class TestAdjustBrightness:
    _CORRECTNESS_BRIGHTNESS_FACTORS = [0.5, 0.0, 1.0, 5.0]
    _DEFAULT_BRIGHTNESS_FACTOR = _CORRECTNESS_BRIGHTNESS_FACTORS[0]

    @pytest.mark.parametrize(
        ("kernel", "make_input"),
        [
            (F.adjust_brightness_image, make_image),
            (F.adjust_brightness_video, make_video),
        ],
    )
    @pytest.mark.parametrize("dtype", [torch.float32, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel(self, kernel, make_input, dtype, device):
        check_kernel(kernel, make_input(dtype=dtype, device=device), brightness_factor=self._DEFAULT_BRIGHTNESS_FACTOR)

    @pytest.mark.parametrize("make_input", [make_image_tensor, make_image_pil, make_image, make_video])
    def test_functional(self, make_input):
        check_functional(F.adjust_brightness, make_input(), brightness_factor=self._DEFAULT_BRIGHTNESS_FACTOR)

    @pytest.mark.parametrize(
        ("kernel", "input_type"),
        [
            (F.adjust_brightness_image, torch.Tensor),
            (F._adjust_brightness_image_pil, PIL.Image.Image),
            (F.adjust_brightness_image, tv_tensors.Image),
            (F.adjust_brightness_video, tv_tensors.Video),
        ],
    )
    def test_functional_signature(self, kernel, input_type):
        check_functional_kernel_signature_match(F.adjust_brightness, kernel=kernel, input_type=input_type)

    @pytest.mark.parametrize("brightness_factor", _CORRECTNESS_BRIGHTNESS_FACTORS)
    def test_image_correctness(self, brightness_factor):
        image = make_image(dtype=torch.uint8, device="cpu")

        actual = F.adjust_brightness(image, brightness_factor=brightness_factor)
        expected = F.to_image(F.adjust_brightness(F.to_pil_image(image), brightness_factor=brightness_factor))

        torch.testing.assert_close(actual, expected)


class TestCutMixMixUp:
    class DummyDataset:
        def __init__(self, size, num_classes):
            self.size = size
            self.num_classes = num_classes
            assert size < num_classes

        def __getitem__(self, idx):
            img = torch.rand(3, 100, 100)
            label = idx  # This ensures all labels in a batch are unique and makes testing easier
            return img, label

        def __len__(self):
            return self.size

    @pytest.mark.parametrize("T", [transforms.CutMix, transforms.MixUp])
    def test_supported_input_structure(self, T):

        batch_size = 32
        num_classes = 100

        dataset = self.DummyDataset(size=batch_size, num_classes=num_classes)

        cutmix_mixup = T(num_classes=num_classes)

        dl = DataLoader(dataset, batch_size=batch_size)

        # Input sanity checks
        img, target = next(iter(dl))
        input_img_size = img.shape[-3:]
        assert isinstance(img, torch.Tensor) and isinstance(target, torch.Tensor)
        assert target.shape == (batch_size,)

        def check_output(img, target):
            assert img.shape == (batch_size, *input_img_size)
            assert target.shape == (batch_size, num_classes)
            torch.testing.assert_close(target.sum(axis=-1), torch.ones(batch_size))
            num_non_zero_labels = (target != 0).sum(axis=-1)
            assert (num_non_zero_labels == 2).all()

        # After Dataloader, as unpacked input
        img, target = next(iter(dl))
        assert target.shape == (batch_size,)
        img, target = cutmix_mixup(img, target)
        check_output(img, target)

        # After Dataloader, as packed input
        packed_from_dl = next(iter(dl))
        assert isinstance(packed_from_dl, list)
        img, target = cutmix_mixup(packed_from_dl)
        check_output(img, target)

        # As collation function. We expect default_collate to be used by users.
        def collate_fn_1(batch):
            return cutmix_mixup(default_collate(batch))

        def collate_fn_2(batch):
            return cutmix_mixup(*default_collate(batch))

        for collate_fn in (collate_fn_1, collate_fn_2):
            dl = DataLoader(dataset, batch_size=batch_size, collate_fn=collate_fn)
            img, target = next(iter(dl))
            check_output(img, target)

    @needs_cuda
    @pytest.mark.parametrize("T", [transforms.CutMix, transforms.MixUp])
    def test_cpu_vs_gpu(self, T):
        num_classes = 10
        batch_size = 3
        H, W = 12, 12

        imgs = torch.rand(batch_size, 3, H, W)
        labels = torch.randint(0, num_classes, (batch_size,))
        cutmix_mixup = T(alpha=0.5, num_classes=num_classes)

        _check_kernel_cuda_vs_cpu(cutmix_mixup, imgs, labels, rtol=None, atol=None)

    @pytest.mark.parametrize("T", [transforms.CutMix, transforms.MixUp])
    def test_error(self, T):

        num_classes = 10
        batch_size = 9

        imgs = torch.rand(batch_size, 3, 12, 12)
        cutmix_mixup = T(alpha=0.5, num_classes=num_classes)

        for input_with_bad_type in (
            F.to_pil_image(imgs[0]),
            tv_tensors.Mask(torch.rand(12, 12)),
            tv_tensors.BoundingBoxes(torch.rand(2, 4), format="XYXY", canvas_size=12),
        ):
            with pytest.raises(ValueError, match="does not support PIL images, "):
                cutmix_mixup(input_with_bad_type)

        with pytest.raises(ValueError, match="Could not infer where the labels are"):
            cutmix_mixup({"img": imgs, "Nothing_else": 3})

        with pytest.raises(ValueError, match="labels tensor should be of shape"):
            # Note: the error message isn't ideal, but that's because the label heuristic found the img as the label
            # It's OK, it's an edge-case. The important thing is that this fails loudly instead of passing silently
            cutmix_mixup(imgs)

        with pytest.raises(ValueError, match="When using the default labels_getter"):
            cutmix_mixup(imgs, "not_a_tensor")

        with pytest.raises(ValueError, match="labels tensor should be of shape"):
            cutmix_mixup(imgs, torch.randint(0, 2, size=(2, 3)))

        with pytest.raises(ValueError, match="Expected a batched input with 4 dims"):
            cutmix_mixup(imgs[None, None], torch.randint(0, num_classes, size=(batch_size,)))

        with pytest.raises(ValueError, match="does not match the batch size of the labels"):
            cutmix_mixup(imgs, torch.randint(0, num_classes, size=(batch_size + 1,)))

        with pytest.raises(ValueError, match="labels tensor should be of shape"):
            # The purpose of this check is more about documenting the current
            # behaviour of what happens on a Compose(), rather than actually
            # asserting the expected behaviour. We may support Compose() in the
            # future, e.g. for 2 consecutive CutMix?
            labels = torch.randint(0, num_classes, size=(batch_size,))
            transforms.Compose([cutmix_mixup, cutmix_mixup])(imgs, labels)


@pytest.mark.parametrize("key", ("labels", "LABELS", "LaBeL", "SOME_WEIRD_KEY_THAT_HAS_LABeL_IN_IT"))
@pytest.mark.parametrize("sample_type", (tuple, list, dict))
def test_labels_getter_default_heuristic(key, sample_type):
    labels = torch.arange(10)
    sample = {key: labels, "another_key": "whatever"}
    if sample_type is not dict:
        sample = sample_type((None, sample, "whatever_again"))
    assert transforms._utils._find_labels_default_heuristic(sample) is labels

    if key.lower() != "labels":
        # If "labels" is in the dict (case-insensitive),
        # it takes precedence over other keys which would otherwise be a match
        d = {key: "something_else", "labels": labels}
        assert transforms._utils._find_labels_default_heuristic(d) is labels


class TestShapeGetters:
    @pytest.mark.parametrize(
        ("kernel", "make_input"),
        [
            (F.get_dimensions_image, make_image_tensor),
            (F._get_dimensions_image_pil, make_image_pil),
            (F.get_dimensions_image, make_image),
            (F.get_dimensions_video, make_video),
        ],
    )
    def test_get_dimensions(self, kernel, make_input):
        size = (10, 10)
        color_space, num_channels = "RGB", 3

        input = make_input(size, color_space=color_space)

        assert kernel(input) == F.get_dimensions(input) == [num_channels, *size]

    @pytest.mark.parametrize(
        ("kernel", "make_input"),
        [
            (F.get_num_channels_image, make_image_tensor),
            (F._get_num_channels_image_pil, make_image_pil),
            (F.get_num_channels_image, make_image),
            (F.get_num_channels_video, make_video),
        ],
    )
    def test_get_num_channels(self, kernel, make_input):
        color_space, num_channels = "RGB", 3

        input = make_input(color_space=color_space)

        assert kernel(input) == F.get_num_channels(input) == num_channels

    @pytest.mark.parametrize(
        ("kernel", "make_input"),
        [
            (F.get_size_image, make_image_tensor),
            (F._get_size_image_pil, make_image_pil),
            (F.get_size_image, make_image),
            (F.get_size_bounding_boxes, make_bounding_boxes),
            (F.get_size_mask, make_detection_mask),
            (F.get_size_mask, make_segmentation_mask),
            (F.get_size_video, make_video),
        ],
    )
    def test_get_size(self, kernel, make_input):
        size = (10, 10)

        input = make_input(size)

        assert kernel(input) == F.get_size(input) == list(size)

    @pytest.mark.parametrize(
        ("kernel", "make_input"),
        [
            (F.get_num_frames_video, make_video_tensor),
            (F.get_num_frames_video, make_video),
        ],
    )
    def test_get_num_frames(self, kernel, make_input):
        num_frames = 4

        input = make_input(num_frames=num_frames)

        assert kernel(input) == F.get_num_frames(input) == num_frames

    @pytest.mark.parametrize(
        ("functional", "make_input"),
        [
            (F.get_dimensions, make_bounding_boxes),
            (F.get_dimensions, make_detection_mask),
            (F.get_dimensions, make_segmentation_mask),
            (F.get_num_channels, make_bounding_boxes),
            (F.get_num_channels, make_detection_mask),
            (F.get_num_channels, make_segmentation_mask),
            (F.get_num_frames, make_image_pil),
            (F.get_num_frames, make_image),
            (F.get_num_frames, make_bounding_boxes),
            (F.get_num_frames, make_detection_mask),
            (F.get_num_frames, make_segmentation_mask),
        ],
    )
    def test_unsupported_types(self, functional, make_input):
        input = make_input()

        with pytest.raises(TypeError, match=re.escape(str(type(input)))):
            functional(input)


class TestRegisterKernel:
    @pytest.mark.parametrize("functional", (F.resize, "resize"))
    def test_register_kernel(self, functional):
        class CustomTVTensor(tv_tensors.TVTensor):
            pass

        kernel_was_called = False

        @F.register_kernel(functional, CustomTVTensor)
        def new_resize(dp, *args, **kwargs):
            nonlocal kernel_was_called
            kernel_was_called = True
            return dp

        t = transforms.Resize(size=(224, 224), antialias=True)

        my_dp = CustomTVTensor(torch.rand(3, 10, 10))
        out = t(my_dp)
        assert out is my_dp
        assert kernel_was_called

        # Sanity check to make sure we didn't override the kernel of other types
        t(torch.rand(3, 10, 10)).shape == (3, 224, 224)
        t(tv_tensors.Image(torch.rand(3, 10, 10))).shape == (3, 224, 224)

    def test_errors(self):
        with pytest.raises(ValueError, match="Could not find functional with name"):
            F.register_kernel("bad_name", tv_tensors.Image)

        with pytest.raises(ValueError, match="Kernels can only be registered on functionals"):
            F.register_kernel(tv_tensors.Image, F.resize)

        with pytest.raises(ValueError, match="Kernels can only be registered for subclasses"):
            F.register_kernel(F.resize, object)

        with pytest.raises(ValueError, match="cannot be registered for the builtin tv_tensor classes"):
            F.register_kernel(F.resize, tv_tensors.Image)(F.resize_image)

        class CustomTVTensor(tv_tensors.TVTensor):
            pass

        def resize_custom_tv_tensor():
            pass

        F.register_kernel(F.resize, CustomTVTensor)(resize_custom_tv_tensor)

        with pytest.raises(ValueError, match="already has a kernel registered for type"):
            F.register_kernel(F.resize, CustomTVTensor)(resize_custom_tv_tensor)


class TestGetKernel:
    # We are using F.resize as functional and the kernels below as proxy. Any other functional / kernels combination
    # would also be fine
    KERNELS = {
        torch.Tensor: F.resize_image,
        PIL.Image.Image: F._resize_image_pil,
        tv_tensors.Image: F.resize_image,
        tv_tensors.BoundingBoxes: F.resize_bounding_boxes,
        tv_tensors.Mask: F.resize_mask,
        tv_tensors.Video: F.resize_video,
    }

    @pytest.mark.parametrize("input_type", [str, int, object])
    def test_unsupported_types(self, input_type):
        with pytest.raises(TypeError, match="supports inputs of type"):
            _get_kernel(F.resize, input_type)

    def test_exact_match(self):
        # We cannot use F.resize together with self.KERNELS mapping here directly here, since this is only the
        # ideal wrapping. Practically, we have an intermediate wrapper layer. Thus, we create a new resize functional
        # here, register the kernels without wrapper, and check the exact matching afterwards.
        def resize_with_pure_kernels():
            pass

        for input_type, kernel in self.KERNELS.items():
            _register_kernel_internal(resize_with_pure_kernels, input_type, tv_tensor_wrapper=False)(kernel)

            assert _get_kernel(resize_with_pure_kernels, input_type) is kernel

    def test_builtin_tv_tensor_subclass(self):
        # We cannot use F.resize together with self.KERNELS mapping here directly here, since this is only the
        # ideal wrapping. Practically, we have an intermediate wrapper layer. Thus, we create a new resize functional
        # here, register the kernels without wrapper, and check if subclasses of our builtin tv_tensors get dispatched
        # to the kernel of the corresponding superclass
        def resize_with_pure_kernels():
            pass

        class MyImage(tv_tensors.Image):
            pass

        class MyBoundingBoxes(tv_tensors.BoundingBoxes):
            pass

        class MyMask(tv_tensors.Mask):
            pass

        class MyVideo(tv_tensors.Video):
            pass

        for custom_tv_tensor_subclass in [
            MyImage,
            MyBoundingBoxes,
            MyMask,
            MyVideo,
        ]:
            builtin_tv_tensor_class = custom_tv_tensor_subclass.__mro__[1]
            builtin_tv_tensor_kernel = self.KERNELS[builtin_tv_tensor_class]
            _register_kernel_internal(resize_with_pure_kernels, builtin_tv_tensor_class, tv_tensor_wrapper=False)(
                builtin_tv_tensor_kernel
            )

            assert _get_kernel(resize_with_pure_kernels, custom_tv_tensor_subclass) is builtin_tv_tensor_kernel

    def test_tv_tensor_subclass(self):
        class MyTVTensor(tv_tensors.TVTensor):
            pass

        with pytest.raises(TypeError, match="supports inputs of type"):
            _get_kernel(F.resize, MyTVTensor)

        def resize_my_tv_tensor():
            pass

        _register_kernel_internal(F.resize, MyTVTensor, tv_tensor_wrapper=False)(resize_my_tv_tensor)

        assert _get_kernel(F.resize, MyTVTensor) is resize_my_tv_tensor

    def test_pil_image_subclass(self):
        opened_image = PIL.Image.open(Path(__file__).parent / "assets" / "encode_jpeg" / "grace_hopper_517x606.jpg")
        loaded_image = opened_image.convert("RGB")

        # check the assumptions
        assert isinstance(opened_image, PIL.Image.Image)
        assert type(opened_image) is not PIL.Image.Image

        assert type(loaded_image) is PIL.Image.Image

        size = [17, 11]
        for image in [opened_image, loaded_image]:
            kernel = _get_kernel(F.resize, type(image))

            output = kernel(image, size=size)

            assert F.get_size(output) == size


class TestPermuteChannels:
    _DEFAULT_PERMUTATION = [2, 0, 1]

    @pytest.mark.parametrize(
        ("kernel", "make_input"),
        [
            (F.permute_channels_image, make_image_tensor),
            # FIXME
            # check_kernel does not support PIL kernel, but it should
            (F.permute_channels_image, make_image),
            (F.permute_channels_video, make_video),
        ],
    )
    @pytest.mark.parametrize("dtype", [torch.float32, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel(self, kernel, make_input, dtype, device):
        check_kernel(kernel, make_input(dtype=dtype, device=device), permutation=self._DEFAULT_PERMUTATION)

    @pytest.mark.parametrize("make_input", [make_image_tensor, make_image_pil, make_image, make_video])
    def test_functional(self, make_input):
        check_functional(F.permute_channels, make_input(), permutation=self._DEFAULT_PERMUTATION)

    @pytest.mark.parametrize(
        ("kernel", "input_type"),
        [
            (F.permute_channels_image, torch.Tensor),
            (F._permute_channels_image_pil, PIL.Image.Image),
            (F.permute_channels_image, tv_tensors.Image),
            (F.permute_channels_video, tv_tensors.Video),
        ],
    )
    def test_functional_signature(self, kernel, input_type):
        check_functional_kernel_signature_match(F.permute_channels, kernel=kernel, input_type=input_type)

    def reference_image_correctness(self, image, permutation):
        channel_images = image.split(1, dim=-3)
        permuted_channel_images = [channel_images[channel_idx] for channel_idx in permutation]
        return tv_tensors.Image(torch.concat(permuted_channel_images, dim=-3))

    @pytest.mark.parametrize("permutation", [[2, 0, 1], [1, 2, 0], [2, 0, 1], [0, 1, 2]])
    @pytest.mark.parametrize("batch_dims", [(), (2,), (2, 1)])
    def test_image_correctness(self, permutation, batch_dims):
        image = make_image(batch_dims=batch_dims)

        actual = F.permute_channels(image, permutation=permutation)
        expected = self.reference_image_correctness(image, permutation=permutation)

        torch.testing.assert_close(actual, expected)


class TestElastic:
    def _make_displacement(self, inpt):
        return torch.rand(
            1,
            *F.get_size(inpt),
            2,
            dtype=torch.float32,
            device=inpt.device if isinstance(inpt, torch.Tensor) else "cpu",
        )

    @param_value_parametrization(
        interpolation=[transforms.InterpolationMode.NEAREST, transforms.InterpolationMode.BILINEAR],
        fill=EXHAUSTIVE_TYPE_FILLS,
    )
    @pytest.mark.parametrize("dtype", [torch.float32, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_image(self, param, value, dtype, device):
        image = make_image_tensor(dtype=dtype, device=device)

        check_kernel(
            F.elastic_image,
            image,
            displacement=self._make_displacement(image),
            **{param: value},
            check_scripted_vs_eager=not (param == "fill" and isinstance(value, (int, float))),
        )

    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("dtype", [torch.float32, torch.int64])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_bounding_boxes(self, format, dtype, device):
        bounding_boxes = make_bounding_boxes(format=format, dtype=dtype, device=device)

        check_kernel(
            F.elastic_bounding_boxes,
            bounding_boxes,
            format=bounding_boxes.format,
            canvas_size=bounding_boxes.canvas_size,
            displacement=self._make_displacement(bounding_boxes),
        )

    @pytest.mark.parametrize("make_mask", [make_segmentation_mask, make_detection_mask])
    def test_kernel_mask(self, make_mask):
        mask = make_mask()
        check_kernel(F.elastic_mask, mask, displacement=self._make_displacement(mask))

    def test_kernel_video(self):
        video = make_video()
        check_kernel(F.elastic_video, video, displacement=self._make_displacement(video))

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    def test_functional(self, make_input):
        input = make_input()
        check_functional(F.elastic, input, displacement=self._make_displacement(input))

    @pytest.mark.parametrize(
        ("kernel", "input_type"),
        [
            (F.elastic_image, torch.Tensor),
            (F._elastic_image_pil, PIL.Image.Image),
            (F.elastic_image, tv_tensors.Image),
            (F.elastic_bounding_boxes, tv_tensors.BoundingBoxes),
            (F.elastic_mask, tv_tensors.Mask),
            (F.elastic_video, tv_tensors.Video),
        ],
    )
    def test_functional_signature(self, kernel, input_type):
        check_functional_kernel_signature_match(F.elastic, kernel=kernel, input_type=input_type)

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    def test_displacement_error(self, make_input):
        input = make_input()

        with pytest.raises(TypeError, match="displacement should be a Tensor"):
            F.elastic(input, displacement=None)

        with pytest.raises(ValueError, match="displacement shape should be"):
            F.elastic(input, displacement=torch.rand(F.get_size(input)))

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    # ElasticTransform needs larger images to avoid the needed internal padding being larger than the actual image
    @pytest.mark.parametrize("size", [(163, 163), (72, 333), (313, 95)])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_transform(self, make_input, size, device):
        check_transform(
            transforms.ElasticTransform(),
            make_input(size, device=device),
            # We updated gaussian blur kernel generation with a faster and numerically more stable version
            check_v1_compatibility=dict(rtol=0, atol=1),
        )


class TestToPureTensor:
    def test_correctness(self):
        input = {
            "img": make_image(),
            "img_tensor": make_image_tensor(),
            "img_pil": make_image_pil(),
            "mask": make_detection_mask(),
            "video": make_video(),
            "bbox": make_bounding_boxes(),
            "str": "str",
        }

        out = transforms.ToPureTensor()(input)

        for input_value, out_value in zip(input.values(), out.values()):
            if isinstance(input_value, tv_tensors.TVTensor):
                assert isinstance(out_value, torch.Tensor) and not isinstance(out_value, tv_tensors.TVTensor)
            else:
                assert isinstance(out_value, type(input_value))


class TestCrop:
    INPUT_SIZE = (21, 11)

    CORRECTNESS_CROP_KWARGS = [
        # center
        dict(top=5, left=5, height=10, width=5),
        # larger than input, i.e. pad
        dict(top=-5, left=-5, height=30, width=20),
        # sides: left, right, top, bottom
        dict(top=-5, left=-5, height=30, width=10),
        dict(top=-5, left=5, height=30, width=10),
        dict(top=-5, left=-5, height=20, width=20),
        dict(top=5, left=-5, height=20, width=20),
        # corners: top-left, top-right, bottom-left, bottom-right
        dict(top=-5, left=-5, height=20, width=10),
        dict(top=-5, left=5, height=20, width=10),
        dict(top=5, left=-5, height=20, width=10),
        dict(top=5, left=5, height=20, width=10),
    ]
    MINIMAL_CROP_KWARGS = CORRECTNESS_CROP_KWARGS[0]

    @pytest.mark.parametrize("kwargs", CORRECTNESS_CROP_KWARGS)
    @pytest.mark.parametrize("dtype", [torch.uint8, torch.float32])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_image(self, kwargs, dtype, device):
        check_kernel(F.crop_image, make_image(self.INPUT_SIZE, dtype=dtype, device=device), **kwargs)

    @pytest.mark.parametrize("kwargs", CORRECTNESS_CROP_KWARGS)
    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("dtype", [torch.float32, torch.int64])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_bounding_box(self, kwargs, format, dtype, device):
        bounding_boxes = make_bounding_boxes(self.INPUT_SIZE, format=format, dtype=dtype, device=device)
        check_kernel(F.crop_bounding_boxes, bounding_boxes, format=format, **kwargs)

    @pytest.mark.parametrize("make_mask", [make_segmentation_mask, make_detection_mask])
    def test_kernel_mask(self, make_mask):
        check_kernel(F.crop_mask, make_mask(self.INPUT_SIZE), **self.MINIMAL_CROP_KWARGS)

    def test_kernel_video(self):
        check_kernel(F.crop_video, make_video(self.INPUT_SIZE), **self.MINIMAL_CROP_KWARGS)

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    def test_functional(self, make_input):
        check_functional(F.crop, make_input(self.INPUT_SIZE), **self.MINIMAL_CROP_KWARGS)

    @pytest.mark.parametrize(
        ("kernel", "input_type"),
        [
            (F.crop_image, torch.Tensor),
            (F._crop_image_pil, PIL.Image.Image),
            (F.crop_image, tv_tensors.Image),
            (F.crop_bounding_boxes, tv_tensors.BoundingBoxes),
            (F.crop_mask, tv_tensors.Mask),
            (F.crop_video, tv_tensors.Video),
        ],
    )
    def test_functional_signature(self, kernel, input_type):
        check_functional_kernel_signature_match(F.crop, kernel=kernel, input_type=input_type)

    @pytest.mark.parametrize("kwargs", CORRECTNESS_CROP_KWARGS)
    def test_functional_image_correctness(self, kwargs):
        image = make_image(self.INPUT_SIZE, dtype=torch.uint8, device="cpu")

        actual = F.crop(image, **kwargs)
        expected = F.to_image(F.crop(F.to_pil_image(image), **kwargs))

        assert_equal(actual, expected)

    @param_value_parametrization(
        size=[(10, 5), (25, 15), (25, 5), (10, 15)],
        fill=EXHAUSTIVE_TYPE_FILLS,
    )
    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    def test_transform(self, param, value, make_input):
        input = make_input(self.INPUT_SIZE)

        if param == "fill":
            if isinstance(input, tv_tensors.Mask) and isinstance(value, (tuple, list)):
                pytest.skip("F.pad_mask doesn't support non-scalar fill.")

            kwargs = dict(
                # 1. size is required
                # 2. the fill parameter only has an affect if we need padding
                size=[s + 4 for s in self.INPUT_SIZE],
                fill=adapt_fill(value, dtype=input.dtype if isinstance(input, torch.Tensor) else torch.uint8),
            )
        else:
            kwargs = {param: value}

        check_transform(
            transforms.RandomCrop(**kwargs, pad_if_needed=True),
            input,
            check_v1_compatibility=param != "fill" or isinstance(value, (int, float)),
        )

    @pytest.mark.parametrize("padding", [1, (1, 1), (1, 1, 1, 1)])
    def test_transform_padding(self, padding):
        inpt = make_image(self.INPUT_SIZE)

        output_size = [s + 2 for s in F.get_size(inpt)]
        transform = transforms.RandomCrop(output_size, padding=padding)

        output = transform(inpt)

        assert F.get_size(output) == output_size

    @pytest.mark.parametrize("padding", [None, 1, (1, 1), (1, 1, 1, 1)])
    def test_transform_insufficient_padding(self, padding):
        inpt = make_image(self.INPUT_SIZE)

        output_size = [s + 3 for s in F.get_size(inpt)]
        transform = transforms.RandomCrop(output_size, padding=padding)

        with pytest.raises(ValueError, match="larger than (padded )?input image size"):
            transform(inpt)

    def test_transform_pad_if_needed(self):
        inpt = make_image(self.INPUT_SIZE)

        output_size = [s * 2 for s in F.get_size(inpt)]
        transform = transforms.RandomCrop(output_size, pad_if_needed=True)

        output = transform(inpt)

        assert F.get_size(output) == output_size

    @param_value_parametrization(
        size=[(10, 5), (25, 15), (25, 5), (10, 15)],
        fill=CORRECTNESS_FILLS,
        padding_mode=["constant", "edge", "reflect", "symmetric"],
    )
    @pytest.mark.parametrize("seed", list(range(5)))
    def test_transform_image_correctness(self, param, value, seed):
        kwargs = {param: value}
        if param != "size":
            # 1. size is required
            # 2. the fill / padding_mode parameters only have an affect if we need padding
            kwargs["size"] = [s + 4 for s in self.INPUT_SIZE]
        if param == "fill":
            kwargs["fill"] = adapt_fill(kwargs["fill"], dtype=torch.uint8)

        transform = transforms.RandomCrop(pad_if_needed=True, **kwargs)

        image = make_image(self.INPUT_SIZE)

        with freeze_rng_state():
            torch.manual_seed(seed)
            actual = transform(image)

            torch.manual_seed(seed)
            expected = F.to_image(transform(F.to_pil_image(image)))

        assert_equal(actual, expected)

    def _reference_crop_bounding_boxes(self, bounding_boxes, *, top, left, height, width):
        affine_matrix = np.array(
            [
                [1, 0, -left],
                [0, 1, -top],
            ],
        )
        return reference_affine_bounding_boxes_helper(
            bounding_boxes, affine_matrix=affine_matrix, new_canvas_size=(height, width)
        )

    @pytest.mark.parametrize("kwargs", CORRECTNESS_CROP_KWARGS)
    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("dtype", [torch.float32, torch.int64])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_functional_bounding_box_correctness(self, kwargs, format, dtype, device):
        bounding_boxes = make_bounding_boxes(self.INPUT_SIZE, format=format, dtype=dtype, device=device)

        actual = F.crop(bounding_boxes, **kwargs)
        expected = self._reference_crop_bounding_boxes(bounding_boxes, **kwargs)

        assert_equal(actual, expected, atol=1, rtol=0)
        assert_equal(F.get_size(actual), F.get_size(expected))

    @pytest.mark.parametrize("output_size", [(17, 11), (11, 17), (11, 11)])
    @pytest.mark.parametrize("format", list(tv_tensors.BoundingBoxFormat))
    @pytest.mark.parametrize("dtype", [torch.float32, torch.int64])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    @pytest.mark.parametrize("seed", list(range(5)))
    def test_transform_bounding_boxes_correctness(self, output_size, format, dtype, device, seed):
        input_size = [s * 2 for s in output_size]
        bounding_boxes = make_bounding_boxes(input_size, format=format, dtype=dtype, device=device)

        transform = transforms.RandomCrop(output_size)

        with freeze_rng_state():
            torch.manual_seed(seed)
            params = transform._get_params([bounding_boxes])
            assert not params.pop("needs_pad")
            del params["padding"]
            assert params.pop("needs_crop")

            torch.manual_seed(seed)
            actual = transform(bounding_boxes)

        expected = self._reference_crop_bounding_boxes(bounding_boxes, **params)

        assert_equal(actual, expected)
        assert_equal(F.get_size(actual), F.get_size(expected))

    def test_errors(self):
        with pytest.raises(ValueError, match="Please provide only two dimensions"):
            transforms.RandomCrop([10, 12, 14])

        with pytest.raises(TypeError, match="Got inappropriate padding arg"):
            transforms.RandomCrop([10, 12], padding="abc")

        with pytest.raises(ValueError, match="Padding must be an int or a 1, 2, or 4"):
            transforms.RandomCrop([10, 12], padding=[-0.7, 0, 0.7])

        with pytest.raises(TypeError, match="Got inappropriate fill arg"):
            transforms.RandomCrop([10, 12], padding=1, fill="abc")

        with pytest.raises(ValueError, match="Padding mode should be either"):
            transforms.RandomCrop([10, 12], padding=1, padding_mode="abc")


class TestErase:
    INPUT_SIZE = (17, 11)
    FUNCTIONAL_KWARGS = dict(
        zip("ijhwv", [2, 2, 10, 8, torch.tensor(0.0, dtype=torch.float32, device="cpu").reshape(-1, 1, 1)])
    )

    @pytest.mark.parametrize("dtype", [torch.float32, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_image(self, dtype, device):
        check_kernel(F.erase_image, make_image(self.INPUT_SIZE, dtype=dtype, device=device), **self.FUNCTIONAL_KWARGS)

    @pytest.mark.parametrize("dtype", [torch.float32, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_kernel_image_inplace(self, dtype, device):
        input = make_image(self.INPUT_SIZE, dtype=dtype, device=device)
        input_version = input._version

        output_out_of_place = F.erase_image(input, **self.FUNCTIONAL_KWARGS)
        assert output_out_of_place.data_ptr() != input.data_ptr()
        assert output_out_of_place is not input

        output_inplace = F.erase_image(input, **self.FUNCTIONAL_KWARGS, inplace=True)
        assert output_inplace.data_ptr() == input.data_ptr()
        assert output_inplace._version > input_version
        assert output_inplace is input

        assert_equal(output_inplace, output_out_of_place)

    def test_kernel_video(self):
        check_kernel(F.erase_video, make_video(self.INPUT_SIZE), **self.FUNCTIONAL_KWARGS)

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_video],
    )
    def test_functional(self, make_input):
        check_functional(F.erase, make_input(), **self.FUNCTIONAL_KWARGS)

    @pytest.mark.parametrize(
        ("kernel", "input_type"),
        [
            (F.erase_image, torch.Tensor),
            (F._erase_image_pil, PIL.Image.Image),
            (F.erase_image, tv_tensors.Image),
            (F.erase_video, tv_tensors.Video),
        ],
    )
    def test_functional_signature(self, kernel, input_type):
        check_functional_kernel_signature_match(F.erase, kernel=kernel, input_type=input_type)

    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_video],
    )
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_transform(self, make_input, device):
        check_transform(transforms.RandomErasing(p=1), make_input(device=device))

    def _reference_erase_image(self, image, *, i, j, h, w, v):
        mask = torch.zeros_like(image, dtype=torch.bool)
        mask[..., i : i + h, j : j + w] = True

        # The broadcasting and type casting logic is handled automagically in the kernel through indexing
        value = torch.broadcast_to(v, (*image.shape[:-2], h, w)).to(image)

        erased_image = torch.empty_like(image)
        erased_image[mask] = value.flatten()
        erased_image[~mask] = image[~mask]

        return erased_image

    @pytest.mark.parametrize("dtype", [torch.float32, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_functional_image_correctness(self, dtype, device):
        image = make_image(dtype=dtype, device=device)

        actual = F.erase(image, **self.FUNCTIONAL_KWARGS)
        expected = self._reference_erase_image(image, **self.FUNCTIONAL_KWARGS)

        assert_equal(actual, expected)

    @param_value_parametrization(
        scale=[(0.1, 0.2), [0.0, 1.0]],
        ratio=[(0.3, 0.7), [0.1, 5.0]],
        value=[0, 0.5, (0, 1, 0), [-0.2, 0.0, 1.3], "random"],
    )
    @pytest.mark.parametrize("dtype", [torch.float32, torch.uint8])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    @pytest.mark.parametrize("seed", list(range(5)))
    def test_transform_image_correctness(self, param, value, dtype, device, seed):
        transform = transforms.RandomErasing(**{param: value}, p=1)

        image = make_image(dtype=dtype, device=device)

        with freeze_rng_state():
            torch.manual_seed(seed)
            # This emulates the random apply check that happens before _get_params is called
            torch.rand(1)
            params = transform._get_params([image])

            torch.manual_seed(seed)
            actual = transform(image)

        expected = self._reference_erase_image(image, **params)

        assert_equal(actual, expected)

    def test_transform_errors(self):
        with pytest.raises(TypeError, match="Argument value should be either a number or str or a sequence"):
            transforms.RandomErasing(value={})

        with pytest.raises(ValueError, match="If value is str, it should be 'random'"):
            transforms.RandomErasing(value="abc")

        with pytest.raises(TypeError, match="Scale should be a sequence"):
            transforms.RandomErasing(scale=123)

        with pytest.raises(TypeError, match="Ratio should be a sequence"):
            transforms.RandomErasing(ratio=123)

        with pytest.raises(ValueError, match="Scale should be between 0 and 1"):
            transforms.RandomErasing(scale=[-1, 2])

        transform = transforms.RandomErasing(value=[1, 2, 3, 4])

        with pytest.raises(ValueError, match="If value is a sequence, it should have either a single value"):
            transform._get_params([make_image()])

    @pytest.mark.parametrize("make_input", [make_bounding_boxes, make_detection_mask])
    def test_transform_passthrough(self, make_input):
        transform = transforms.RandomErasing(p=1)

        input = make_input(self.INPUT_SIZE)

        with pytest.warns(UserWarning, match="currently passing through inputs of type"):
            # RandomErasing requires an image or video to be present
            _, output = transform(make_image(self.INPUT_SIZE), input)

        assert output is input


class TestGaussianBlur:
    @pytest.mark.parametrize(
        "make_input",
        [make_image_tensor, make_image_pil, make_image, make_bounding_boxes, make_segmentation_mask, make_video],
    )
    @pytest.mark.parametrize("device", cpu_and_cuda())
    @pytest.mark.parametrize("sigma", [5, (0.5, 2)])
    def test_transform(self, make_input, device, sigma):
        check_transform(transforms.GaussianBlur(kernel_size=3, sigma=sigma), make_input(device=device))

    def test_assertions(self):
        with pytest.raises(ValueError, match="Kernel size should be a tuple/list of two integers"):
            transforms.GaussianBlur([10, 12, 14])

        with pytest.raises(ValueError, match="Kernel size value should be an odd and positive number"):
            transforms.GaussianBlur(4)

        with pytest.raises(ValueError, match="If sigma is a sequence its length should be 1 or 2. Got 3"):
            transforms.GaussianBlur(3, sigma=[1, 2, 3])

        with pytest.raises(ValueError, match="sigma values should be positive and of the form"):
            transforms.GaussianBlur(3, sigma=-1.0)

        with pytest.raises(ValueError, match="sigma values should be positive and of the form"):
            transforms.GaussianBlur(3, sigma=[2.0, 1.0])

        with pytest.raises(TypeError, match="sigma should be a number or a sequence of numbers"):
            transforms.GaussianBlur(3, sigma={})

    @pytest.mark.parametrize("sigma", [10.0, [10.0, 12.0], (10, 12.0), [10]])
    def test__get_params(self, sigma):
        transform = transforms.GaussianBlur(3, sigma=sigma)
        params = transform._get_params([])

        if isinstance(sigma, float):
            assert params["sigma"][0] == params["sigma"][1] == sigma
        elif isinstance(sigma, list) and len(sigma) == 1:
            assert params["sigma"][0] == params["sigma"][1] == sigma[0]
        else:
            assert sigma[0] <= params["sigma"][0] <= sigma[1]
            assert sigma[0] <= params["sigma"][1] <= sigma[1]
