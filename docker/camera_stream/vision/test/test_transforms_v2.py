import itertools
import pathlib
import pickle
import random
import warnings

import numpy as np

import PIL.Image
import pytest
import torch
import torchvision.transforms.v2 as transforms

from common_utils import assert_equal, cpu_and_cuda
from torch.utils._pytree import tree_flatten, tree_unflatten
from torchvision import tv_tensors
from torchvision.ops.boxes import box_iou
from torchvision.transforms.functional import to_pil_image
from torchvision.transforms.v2 import functional as F
from torchvision.transforms.v2._utils import check_type, is_pure_tensor, query_chw
from transforms_v2_legacy_utils import (
    make_bounding_boxes,
    make_detection_mask,
    make_image,
    make_images,
    make_multiple_bounding_boxes,
    make_segmentation_mask,
    make_video,
    make_videos,
)


def make_vanilla_tensor_images(*args, **kwargs):
    for image in make_images(*args, **kwargs):
        if image.ndim > 3:
            continue
        yield image.data


def make_pil_images(*args, **kwargs):
    for image in make_vanilla_tensor_images(*args, **kwargs):
        yield to_pil_image(image)


def make_vanilla_tensor_bounding_boxes(*args, **kwargs):
    for bounding_boxes in make_multiple_bounding_boxes(*args, **kwargs):
        yield bounding_boxes.data


def parametrize(transforms_with_inputs):
    return pytest.mark.parametrize(
        ("transform", "input"),
        [
            pytest.param(
                transform,
                input,
                id=f"{type(transform).__name__}-{type(input).__module__}.{type(input).__name__}-{idx}",
            )
            for transform, inputs in transforms_with_inputs
            for idx, input in enumerate(inputs)
        ],
    )


def auto_augment_adapter(transform, input, device):
    adapted_input = {}
    image_or_video_found = False
    for key, value in input.items():
        if isinstance(value, (tv_tensors.BoundingBoxes, tv_tensors.Mask)):
            # AA transforms don't support bounding boxes or masks
            continue
        elif check_type(value, (tv_tensors.Image, tv_tensors.Video, is_pure_tensor, PIL.Image.Image)):
            if image_or_video_found:
                # AA transforms only support a single image or video
                continue
            image_or_video_found = True
        adapted_input[key] = value
    return adapted_input


def linear_transformation_adapter(transform, input, device):
    flat_inputs = list(input.values())
    c, h, w = query_chw(
        [
            item
            for item, needs_transform in zip(flat_inputs, transforms.Transform()._needs_transform_list(flat_inputs))
            if needs_transform
        ]
    )
    num_elements = c * h * w
    transform.transformation_matrix = torch.randn((num_elements, num_elements), device=device)
    transform.mean_vector = torch.randn((num_elements,), device=device)
    return {key: value for key, value in input.items() if not isinstance(value, PIL.Image.Image)}


def normalize_adapter(transform, input, device):
    adapted_input = {}
    for key, value in input.items():
        if isinstance(value, PIL.Image.Image):
            # normalize doesn't support PIL images
            continue
        elif check_type(value, (tv_tensors.Image, tv_tensors.Video, is_pure_tensor)):
            # normalize doesn't support integer images
            value = F.to_dtype(value, torch.float32, scale=True)
        adapted_input[key] = value
    return adapted_input


class TestSmoke:
    @pytest.mark.parametrize(
        ("transform", "adapter"),
        [
            (transforms.RandomErasing(p=1.0), None),
            (transforms.AugMix(), auto_augment_adapter),
            (transforms.AutoAugment(), auto_augment_adapter),
            (transforms.RandAugment(), auto_augment_adapter),
            (transforms.TrivialAugmentWide(), auto_augment_adapter),
            (transforms.ColorJitter(brightness=0.1, contrast=0.2, saturation=0.3, hue=0.15), None),
            (transforms.Grayscale(), None),
            (transforms.RandomAdjustSharpness(sharpness_factor=0.5, p=1.0), None),
            (transforms.RandomAutocontrast(p=1.0), None),
            (transforms.RandomEqualize(p=1.0), None),
            (transforms.RandomGrayscale(p=1.0), None),
            (transforms.RandomInvert(p=1.0), None),
            (transforms.RandomChannelPermutation(), None),
            (transforms.RandomPhotometricDistort(p=1.0), None),
            (transforms.RandomPosterize(bits=4, p=1.0), None),
            (transforms.RandomSolarize(threshold=0.5, p=1.0), None),
            (transforms.CenterCrop([16, 16]), None),
            (transforms.ElasticTransform(sigma=1.0), None),
            (transforms.Pad(4), None),
            (transforms.RandomAffine(degrees=30.0), None),
            (transforms.RandomCrop([16, 16], pad_if_needed=True), None),
            (transforms.RandomHorizontalFlip(p=1.0), None),
            (transforms.RandomPerspective(p=1.0), None),
            (transforms.RandomResize(min_size=10, max_size=20, antialias=True), None),
            (transforms.RandomResizedCrop([16, 16], antialias=True), None),
            (transforms.RandomRotation(degrees=30), None),
            (transforms.RandomShortestSize(min_size=10, antialias=True), None),
            (transforms.RandomVerticalFlip(p=1.0), None),
            (transforms.RandomZoomOut(p=1.0), None),
            (transforms.Resize([16, 16], antialias=True), None),
            (transforms.ScaleJitter((16, 16), scale_range=(0.8, 1.2), antialias=True), None),
            (transforms.ClampBoundingBoxes(), None),
            (transforms.ConvertBoundingBoxFormat(tv_tensors.BoundingBoxFormat.CXCYWH), None),
            (transforms.ConvertImageDtype(), None),
            (transforms.GaussianBlur(kernel_size=3), None),
            (
                transforms.LinearTransformation(
                    # These are just dummy values that will be filled by the adapter. We can't define them upfront,
                    # because for we neither know the spatial size nor the device at this point
                    transformation_matrix=torch.empty((1, 1)),
                    mean_vector=torch.empty((1,)),
                ),
                linear_transformation_adapter,
            ),
            (transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]), normalize_adapter),
            (transforms.ToDtype(torch.float64), None),
            (transforms.UniformTemporalSubsample(num_samples=2), None),
        ],
        ids=lambda transform: type(transform).__name__,
    )
    @pytest.mark.parametrize("container_type", [dict, list, tuple])
    @pytest.mark.parametrize(
        "image_or_video",
        [
            make_image(),
            make_video(),
            next(make_pil_images(color_spaces=["RGB"])),
            next(make_vanilla_tensor_images()),
        ],
    )
    @pytest.mark.parametrize("de_serialize", [lambda t: t, lambda t: pickle.loads(pickle.dumps(t))])
    @pytest.mark.parametrize("device", cpu_and_cuda())
    def test_common(self, transform, adapter, container_type, image_or_video, de_serialize, device):
        transform = de_serialize(transform)

        canvas_size = F.get_size(image_or_video)
        input = dict(
            image_or_video=image_or_video,
            image_tv_tensor=make_image(size=canvas_size),
            video_tv_tensor=make_video(size=canvas_size),
            image_pil=next(make_pil_images(sizes=[canvas_size], color_spaces=["RGB"])),
            bounding_boxes_xyxy=make_bounding_boxes(
                format=tv_tensors.BoundingBoxFormat.XYXY, canvas_size=canvas_size, batch_dims=(3,)
            ),
            bounding_boxes_xywh=make_bounding_boxes(
                format=tv_tensors.BoundingBoxFormat.XYWH, canvas_size=canvas_size, batch_dims=(4,)
            ),
            bounding_boxes_cxcywh=make_bounding_boxes(
                format=tv_tensors.BoundingBoxFormat.CXCYWH, canvas_size=canvas_size, batch_dims=(5,)
            ),
            bounding_boxes_degenerate_xyxy=tv_tensors.BoundingBoxes(
                [
                    [0, 0, 0, 0],  # no height or width
                    [0, 0, 0, 1],  # no height
                    [0, 0, 1, 0],  # no width
                    [2, 0, 1, 1],  # x1 > x2, y1 < y2
                    [0, 2, 1, 1],  # x1 < x2, y1 > y2
                    [2, 2, 1, 1],  # x1 > x2, y1 > y2
                ],
                format=tv_tensors.BoundingBoxFormat.XYXY,
                canvas_size=canvas_size,
            ),
            bounding_boxes_degenerate_xywh=tv_tensors.BoundingBoxes(
                [
                    [0, 0, 0, 0],  # no height or width
                    [0, 0, 0, 1],  # no height
                    [0, 0, 1, 0],  # no width
                    [0, 0, 1, -1],  # negative height
                    [0, 0, -1, 1],  # negative width
                    [0, 0, -1, -1],  # negative height and width
                ],
                format=tv_tensors.BoundingBoxFormat.XYWH,
                canvas_size=canvas_size,
            ),
            bounding_boxes_degenerate_cxcywh=tv_tensors.BoundingBoxes(
                [
                    [0, 0, 0, 0],  # no height or width
                    [0, 0, 0, 1],  # no height
                    [0, 0, 1, 0],  # no width
                    [0, 0, 1, -1],  # negative height
                    [0, 0, -1, 1],  # negative width
                    [0, 0, -1, -1],  # negative height and width
                ],
                format=tv_tensors.BoundingBoxFormat.CXCYWH,
                canvas_size=canvas_size,
            ),
            detection_mask=make_detection_mask(size=canvas_size),
            segmentation_mask=make_segmentation_mask(size=canvas_size),
            int=0,
            float=0.0,
            bool=True,
            none=None,
            str="str",
            path=pathlib.Path.cwd(),
            object=object(),
            tensor=torch.empty(5),
            array=np.empty(5),
        )
        if adapter is not None:
            input = adapter(transform, input, device)

        if container_type in {tuple, list}:
            input = container_type(input.values())

        input_flat, input_spec = tree_flatten(input)
        input_flat = [item.to(device) if isinstance(item, torch.Tensor) else item for item in input_flat]
        input = tree_unflatten(input_flat, input_spec)

        torch.manual_seed(0)
        output = transform(input)
        output_flat, output_spec = tree_flatten(output)

        assert output_spec == input_spec

        for output_item, input_item, should_be_transformed in zip(
            output_flat, input_flat, transforms.Transform()._needs_transform_list(input_flat)
        ):
            if should_be_transformed:
                assert type(output_item) is type(input_item)
            else:
                assert output_item is input_item

            if isinstance(input_item, tv_tensors.BoundingBoxes) and not isinstance(
                transform, transforms.ConvertBoundingBoxFormat
            ):
                assert output_item.format == input_item.format

        # Enforce that the transform does not turn a degenerate box marked by RandomIoUCrop (or any other future
        # transform that does this), back into a valid one.
        # TODO: we should test that against all degenerate boxes above
        for format in list(tv_tensors.BoundingBoxFormat):
            sample = dict(
                boxes=tv_tensors.BoundingBoxes([[0, 0, 0, 0]], format=format, canvas_size=(224, 244)),
                labels=torch.tensor([3]),
            )
            assert transforms.SanitizeBoundingBoxes()(sample)["boxes"].shape == (0, 4)

    @parametrize(
        [
            (
                transform,
                itertools.chain.from_iterable(
                    fn(
                        color_spaces=[
                            "GRAY",
                            "RGB",
                        ],
                        dtypes=[torch.uint8],
                        extra_dims=[(), (4,)],
                        **(dict(num_frames=[3]) if fn is make_videos else dict()),
                    )
                    for fn in [
                        make_images,
                        make_vanilla_tensor_images,
                        make_pil_images,
                        make_videos,
                    ]
                ),
            )
            for transform in (
                transforms.RandAugment(),
                transforms.TrivialAugmentWide(),
                transforms.AutoAugment(),
                transforms.AugMix(),
            )
        ]
    )
    def test_auto_augment(self, transform, input):
        transform(input)

    @parametrize(
        [
            (
                transforms.Normalize(mean=[0.0, 0.0, 0.0], std=[1.0, 1.0, 1.0]),
                itertools.chain.from_iterable(
                    fn(color_spaces=["RGB"], dtypes=[torch.float32])
                    for fn in [
                        make_images,
                        make_vanilla_tensor_images,
                        make_videos,
                    ]
                ),
            ),
        ]
    )
    def test_normalize(self, transform, input):
        transform(input)

    @parametrize(
        [
            (
                transforms.RandomResizedCrop([16, 16], antialias=True),
                itertools.chain(
                    make_images(extra_dims=[(4,)]),
                    make_vanilla_tensor_images(),
                    make_pil_images(),
                    make_videos(extra_dims=[()]),
                ),
            )
        ]
    )
    def test_random_resized_crop(self, transform, input):
        transform(input)


@pytest.mark.parametrize(
    "flat_inputs",
    itertools.permutations(
        [
            next(make_vanilla_tensor_images()),
            next(make_vanilla_tensor_images()),
            next(make_pil_images()),
            make_image(),
            next(make_videos()),
        ],
        3,
    ),
)
def test_pure_tensor_heuristic(flat_inputs):
    def split_on_pure_tensor(to_split):
        # This takes a sequence that is structurally aligned with `flat_inputs` and splits its items into three parts:
        # 1. The first pure tensor. If none is present, this will be `None`
        # 2. A list of the remaining pure tensors
        # 3. A list of all other items
        pure_tensors = []
        others = []
        # Splitting always happens on the original `flat_inputs` to avoid any erroneous type changes by the transform to
        # affect the splitting.
        for item, inpt in zip(to_split, flat_inputs):
            (pure_tensors if is_pure_tensor(inpt) else others).append(item)
        return pure_tensors[0] if pure_tensors else None, pure_tensors[1:], others

    class CopyCloneTransform(transforms.Transform):
        def _transform(self, inpt, params):
            return inpt.clone() if isinstance(inpt, torch.Tensor) else inpt.copy()

        @staticmethod
        def was_applied(output, inpt):
            identity = output is inpt
            if identity:
                return False

            # Make sure nothing fishy is going on
            assert_equal(output, inpt)
            return True

    first_pure_tensor_input, other_pure_tensor_inputs, other_inputs = split_on_pure_tensor(flat_inputs)

    transform = CopyCloneTransform()
    transformed_sample = transform(flat_inputs)

    first_pure_tensor_output, other_pure_tensor_outputs, other_outputs = split_on_pure_tensor(transformed_sample)

    if first_pure_tensor_input is not None:
        if other_inputs:
            assert not transform.was_applied(first_pure_tensor_output, first_pure_tensor_input)
        else:
            assert transform.was_applied(first_pure_tensor_output, first_pure_tensor_input)

    for output, inpt in zip(other_pure_tensor_outputs, other_pure_tensor_inputs):
        assert not transform.was_applied(output, inpt)

    for input, output in zip(other_inputs, other_outputs):
        assert transform.was_applied(output, input)


class TestPad:
    def test_assertions(self):
        with pytest.raises(TypeError, match="Got inappropriate padding arg"):
            transforms.Pad("abc")

        with pytest.raises(ValueError, match="Padding must be an int or a 1, 2, or 4"):
            transforms.Pad([-0.7, 0, 0.7])

        with pytest.raises(TypeError, match="Got inappropriate fill arg"):
            transforms.Pad(12, fill="abc")

        with pytest.raises(ValueError, match="Padding mode should be either"):
            transforms.Pad(12, padding_mode="abc")


class TestRandomZoomOut:
    def test_assertions(self):
        with pytest.raises(TypeError, match="Got inappropriate fill arg"):
            transforms.RandomZoomOut(fill="abc")

        with pytest.raises(TypeError, match="should be a sequence of length"):
            transforms.RandomZoomOut(0, side_range=0)

        with pytest.raises(ValueError, match="Invalid canvas side range"):
            transforms.RandomZoomOut(0, side_range=[4.0, 1.0])

    @pytest.mark.parametrize("fill", [0, [1, 2, 3], (2, 3, 4)])
    @pytest.mark.parametrize("side_range", [(1.0, 4.0), [2.0, 5.0]])
    def test__get_params(self, fill, side_range):
        transform = transforms.RandomZoomOut(fill=fill, side_range=side_range)

        h, w = size = (24, 32)
        image = make_image(size)

        params = transform._get_params([image])

        assert len(params["padding"]) == 4
        assert 0 <= params["padding"][0] <= (side_range[1] - 1) * w
        assert 0 <= params["padding"][1] <= (side_range[1] - 1) * h
        assert 0 <= params["padding"][2] <= (side_range[1] - 1) * w
        assert 0 <= params["padding"][3] <= (side_range[1] - 1) * h


class TestRandomPerspective:
    def test_assertions(self):
        with pytest.raises(ValueError, match="Argument distortion_scale value should be between 0 and 1"):
            transforms.RandomPerspective(distortion_scale=-1.0)

        with pytest.raises(TypeError, match="Got inappropriate fill arg"):
            transforms.RandomPerspective(0.5, fill="abc")

    def test__get_params(self):
        dscale = 0.5
        transform = transforms.RandomPerspective(dscale)

        image = make_image((24, 32))

        params = transform._get_params([image])

        assert "coefficients" in params
        assert len(params["coefficients"]) == 8


class TestElasticTransform:
    def test_assertions(self):

        with pytest.raises(TypeError, match="alpha should be a number or a sequence of numbers"):
            transforms.ElasticTransform({})

        with pytest.raises(ValueError, match="alpha is a sequence its length should be 1 or 2"):
            transforms.ElasticTransform([1.0, 2.0, 3.0])

        with pytest.raises(TypeError, match="sigma should be a number or a sequence of numbers"):
            transforms.ElasticTransform(1.0, {})

        with pytest.raises(ValueError, match="sigma is a sequence its length should be 1 or 2"):
            transforms.ElasticTransform(1.0, [1.0, 2.0, 3.0])

        with pytest.raises(TypeError, match="Got inappropriate fill arg"):
            transforms.ElasticTransform(1.0, 2.0, fill="abc")

    def test__get_params(self):
        alpha = 2.0
        sigma = 3.0
        transform = transforms.ElasticTransform(alpha, sigma)

        h, w = size = (24, 32)
        image = make_image(size)

        params = transform._get_params([image])

        displacement = params["displacement"]
        assert displacement.shape == (1, h, w, 2)
        assert (-alpha / w <= displacement[0, ..., 0]).all() and (displacement[0, ..., 0] <= alpha / w).all()
        assert (-alpha / h <= displacement[0, ..., 1]).all() and (displacement[0, ..., 1] <= alpha / h).all()


class TestTransform:
    @pytest.mark.parametrize(
        "inpt_type",
        [torch.Tensor, PIL.Image.Image, tv_tensors.Image, np.ndarray, tv_tensors.BoundingBoxes, str, int],
    )
    def test_check_transformed_types(self, inpt_type, mocker):
        # This test ensures that we correctly handle which types to transform and which to bypass
        t = transforms.Transform()
        inpt = mocker.MagicMock(spec=inpt_type)

        if inpt_type in (np.ndarray, str, int):
            output = t(inpt)
            assert output is inpt
        else:
            with pytest.raises(NotImplementedError):
                t(inpt)


class TestToImage:
    @pytest.mark.parametrize(
        "inpt_type",
        [torch.Tensor, PIL.Image.Image, tv_tensors.Image, np.ndarray, tv_tensors.BoundingBoxes, str, int],
    )
    def test__transform(self, inpt_type, mocker):
        fn = mocker.patch(
            "torchvision.transforms.v2.functional.to_image",
            return_value=torch.rand(1, 3, 8, 8),
        )

        inpt = mocker.MagicMock(spec=inpt_type)
        transform = transforms.ToImage()
        transform(inpt)
        if inpt_type in (tv_tensors.BoundingBoxes, tv_tensors.Image, str, int):
            assert fn.call_count == 0
        else:
            fn.assert_called_once_with(inpt)


class TestToPILImage:
    @pytest.mark.parametrize(
        "inpt_type",
        [torch.Tensor, PIL.Image.Image, tv_tensors.Image, np.ndarray, tv_tensors.BoundingBoxes, str, int],
    )
    def test__transform(self, inpt_type, mocker):
        fn = mocker.patch("torchvision.transforms.v2.functional.to_pil_image")

        inpt = mocker.MagicMock(spec=inpt_type)
        transform = transforms.ToPILImage()
        transform(inpt)
        if inpt_type in (PIL.Image.Image, tv_tensors.BoundingBoxes, str, int):
            assert fn.call_count == 0
        else:
            fn.assert_called_once_with(inpt, mode=transform.mode)


class TestToTensor:
    @pytest.mark.parametrize(
        "inpt_type",
        [torch.Tensor, PIL.Image.Image, tv_tensors.Image, np.ndarray, tv_tensors.BoundingBoxes, str, int],
    )
    def test__transform(self, inpt_type, mocker):
        fn = mocker.patch("torchvision.transforms.functional.to_tensor")

        inpt = mocker.MagicMock(spec=inpt_type)
        with pytest.warns(UserWarning, match="deprecated and will be removed"):
            transform = transforms.ToTensor()
        transform(inpt)
        if inpt_type in (tv_tensors.Image, torch.Tensor, tv_tensors.BoundingBoxes, str, int):
            assert fn.call_count == 0
        else:
            fn.assert_called_once_with(inpt)


class TestContainers:
    @pytest.mark.parametrize("transform_cls", [transforms.Compose, transforms.RandomChoice, transforms.RandomOrder])
    def test_assertions(self, transform_cls):
        with pytest.raises(TypeError, match="Argument transforms should be a sequence of callables"):
            transform_cls(transforms.RandomCrop(28))

    @pytest.mark.parametrize("transform_cls", [transforms.Compose, transforms.RandomChoice, transforms.RandomOrder])
    @pytest.mark.parametrize(
        "trfms",
        [
            [transforms.Pad(2), transforms.RandomCrop(28)],
            [lambda x: 2.0 * x, transforms.Pad(2), transforms.RandomCrop(28)],
            [transforms.Pad(2), lambda x: 2.0 * x, transforms.RandomCrop(28)],
        ],
    )
    def test_ctor(self, transform_cls, trfms):
        c = transform_cls(trfms)
        inpt = torch.rand(1, 3, 32, 32)
        output = c(inpt)
        assert isinstance(output, torch.Tensor)
        assert output.ndim == 4


class TestRandomChoice:
    def test_assertions(self):
        with pytest.raises(ValueError, match="Length of p doesn't match the number of transforms"):
            transforms.RandomChoice([transforms.Pad(2), transforms.RandomCrop(28)], p=[1])


class TestRandomIoUCrop:
    @pytest.mark.parametrize("device", cpu_and_cuda())
    @pytest.mark.parametrize("options", [[0.5, 0.9], [2.0]])
    def test__get_params(self, device, options):
        orig_h, orig_w = size = (24, 32)
        image = make_image(size)
        bboxes = tv_tensors.BoundingBoxes(
            torch.tensor([[1, 1, 10, 10], [20, 20, 23, 23], [1, 20, 10, 23], [20, 1, 23, 10]]),
            format="XYXY",
            canvas_size=size,
            device=device,
        )
        sample = [image, bboxes]

        transform = transforms.RandomIoUCrop(sampler_options=options)

        n_samples = 5
        for _ in range(n_samples):

            params = transform._get_params(sample)

            if options == [2.0]:
                assert len(params) == 0
                return

            assert len(params["is_within_crop_area"]) > 0
            assert params["is_within_crop_area"].dtype == torch.bool

            assert int(transform.min_scale * orig_h) <= params["height"] <= int(transform.max_scale * orig_h)
            assert int(transform.min_scale * orig_w) <= params["width"] <= int(transform.max_scale * orig_w)

            left, top = params["left"], params["top"]
            new_h, new_w = params["height"], params["width"]
            ious = box_iou(
                bboxes,
                torch.tensor([[left, top, left + new_w, top + new_h]], dtype=bboxes.dtype, device=bboxes.device),
            )
            assert ious.max() >= options[0] or ious.max() >= options[1], f"{ious} vs {options}"

    def test__transform_empty_params(self, mocker):
        transform = transforms.RandomIoUCrop(sampler_options=[2.0])
        image = tv_tensors.Image(torch.rand(1, 3, 4, 4))
        bboxes = tv_tensors.BoundingBoxes(torch.tensor([[1, 1, 2, 2]]), format="XYXY", canvas_size=(4, 4))
        label = torch.tensor([1])
        sample = [image, bboxes, label]
        # Let's mock transform._get_params to control the output:
        transform._get_params = mocker.MagicMock(return_value={})
        output = transform(sample)
        torch.testing.assert_close(output, sample)

    def test_forward_assertion(self):
        transform = transforms.RandomIoUCrop()
        with pytest.raises(
            TypeError,
            match="requires input sample to contain tensor or PIL images and bounding boxes",
        ):
            transform(torch.tensor(0))

    def test__transform(self, mocker):
        transform = transforms.RandomIoUCrop()

        size = (32, 24)
        image = make_image(size)
        bboxes = make_bounding_boxes(format="XYXY", canvas_size=size, batch_dims=(6,))
        masks = make_detection_mask(size, num_objects=6)

        sample = [image, bboxes, masks]

        is_within_crop_area = torch.tensor([0, 1, 0, 1, 0, 1], dtype=torch.bool)

        params = dict(top=1, left=2, height=12, width=12, is_within_crop_area=is_within_crop_area)
        transform._get_params = mocker.MagicMock(return_value=params)
        output = transform(sample)

        # check number of bboxes vs number of labels:
        output_bboxes = output[1]
        assert isinstance(output_bboxes, tv_tensors.BoundingBoxes)
        assert (output_bboxes[~is_within_crop_area] == 0).all()

        output_masks = output[2]
        assert isinstance(output_masks, tv_tensors.Mask)


class TestScaleJitter:
    def test__get_params(self):
        canvas_size = (24, 32)
        target_size = (16, 12)
        scale_range = (0.5, 1.5)

        transform = transforms.ScaleJitter(target_size=target_size, scale_range=scale_range)

        sample = make_image(canvas_size)

        n_samples = 5
        for _ in range(n_samples):

            params = transform._get_params([sample])

            assert "size" in params
            size = params["size"]

            assert isinstance(size, tuple) and len(size) == 2
            height, width = size

            r_min = min(target_size[1] / canvas_size[0], target_size[0] / canvas_size[1]) * scale_range[0]
            r_max = min(target_size[1] / canvas_size[0], target_size[0] / canvas_size[1]) * scale_range[1]

            assert int(canvas_size[0] * r_min) <= height <= int(canvas_size[0] * r_max)
            assert int(canvas_size[1] * r_min) <= width <= int(canvas_size[1] * r_max)


class TestRandomShortestSize:
    @pytest.mark.parametrize("min_size,max_size", [([5, 9], 20), ([5, 9], None)])
    def test__get_params(self, min_size, max_size):
        canvas_size = (3, 10)

        transform = transforms.RandomShortestSize(min_size=min_size, max_size=max_size, antialias=True)

        sample = make_image(canvas_size)
        params = transform._get_params([sample])

        assert "size" in params
        size = params["size"]

        assert isinstance(size, tuple) and len(size) == 2

        longer = max(size)
        shorter = min(size)
        if max_size is not None:
            assert longer <= max_size
            assert shorter <= max_size
        else:
            assert shorter in min_size


class TestLinearTransformation:
    def test_assertions(self):
        with pytest.raises(ValueError, match="transformation_matrix should be square"):
            transforms.LinearTransformation(torch.rand(2, 3), torch.rand(5))

        with pytest.raises(ValueError, match="mean_vector should have the same length"):
            transforms.LinearTransformation(torch.rand(3, 3), torch.rand(5))

    @pytest.mark.parametrize(
        "inpt",
        [
            122 * torch.ones(1, 3, 8, 8),
            122.0 * torch.ones(1, 3, 8, 8),
            tv_tensors.Image(122 * torch.ones(1, 3, 8, 8)),
            PIL.Image.new("RGB", (8, 8), (122, 122, 122)),
        ],
    )
    def test__transform(self, inpt):

        v = 121 * torch.ones(3 * 8 * 8)
        m = torch.ones(3 * 8 * 8, 3 * 8 * 8)
        transform = transforms.LinearTransformation(m, v)

        if isinstance(inpt, PIL.Image.Image):
            with pytest.raises(TypeError, match="does not support PIL images"):
                transform(inpt)
        else:
            output = transform(inpt)
            assert isinstance(output, torch.Tensor)
            assert output.unique() == 3 * 8 * 8
            assert output.dtype == inpt.dtype


class TestRandomResize:
    def test__get_params(self):
        min_size = 3
        max_size = 6

        transform = transforms.RandomResize(min_size=min_size, max_size=max_size, antialias=True)

        for _ in range(10):
            params = transform._get_params([])

            assert isinstance(params["size"], list) and len(params["size"]) == 1
            size = params["size"][0]

            assert min_size <= size < max_size


class TestUniformTemporalSubsample:
    @pytest.mark.parametrize(
        "inpt",
        [
            torch.zeros(10, 3, 8, 8),
            torch.zeros(1, 10, 3, 8, 8),
            tv_tensors.Video(torch.zeros(1, 10, 3, 8, 8)),
        ],
    )
    def test__transform(self, inpt):
        num_samples = 5
        transform = transforms.UniformTemporalSubsample(num_samples)

        output = transform(inpt)
        assert type(output) is type(inpt)
        assert output.shape[-4] == num_samples
        assert output.dtype == inpt.dtype


# TODO: remove this test in 0.17 when the default of antialias changes to True
def test_antialias_warning():
    pil_img = PIL.Image.new("RGB", size=(10, 10), color=127)
    tensor_img = torch.randint(0, 256, size=(3, 10, 10), dtype=torch.uint8)
    tensor_video = torch.randint(0, 256, size=(2, 3, 10, 10), dtype=torch.uint8)

    match = "The default value of the antialias parameter"
    with pytest.warns(UserWarning, match=match):
        transforms.RandomResizedCrop((20, 20))(tensor_img)
    with pytest.warns(UserWarning, match=match):
        transforms.ScaleJitter((20, 20))(tensor_img)
    with pytest.warns(UserWarning, match=match):
        transforms.RandomShortestSize((20, 20))(tensor_img)
    with pytest.warns(UserWarning, match=match):
        transforms.RandomResize(10, 20)(tensor_img)

    with pytest.warns(UserWarning, match=match):
        F.resized_crop(tv_tensors.Image(tensor_img), 0, 0, 10, 10, (20, 20))

    with pytest.warns(UserWarning, match=match):
        F.resize(tv_tensors.Video(tensor_video), (20, 20))
    with pytest.warns(UserWarning, match=match):
        F.resized_crop(tv_tensors.Video(tensor_video), 0, 0, 10, 10, (20, 20))

    with warnings.catch_warnings():
        warnings.simplefilter("error")
        transforms.RandomResizedCrop((20, 20))(pil_img)
        transforms.ScaleJitter((20, 20))(pil_img)
        transforms.RandomShortestSize((20, 20))(pil_img)
        transforms.RandomResize(10, 20)(pil_img)

        transforms.RandomResizedCrop((20, 20), antialias=True)(tensor_img)
        transforms.ScaleJitter((20, 20), antialias=True)(tensor_img)
        transforms.RandomShortestSize((20, 20), antialias=True)(tensor_img)
        transforms.RandomResize(10, 20, antialias=True)(tensor_img)

        F.resized_crop(tv_tensors.Image(tensor_img), 0, 0, 10, 10, (20, 20), antialias=True)
        F.resized_crop(tv_tensors.Video(tensor_video), 0, 0, 10, 10, (20, 20), antialias=True)


@pytest.mark.parametrize("image_type", (PIL.Image, torch.Tensor, tv_tensors.Image))
@pytest.mark.parametrize("label_type", (torch.Tensor, int))
@pytest.mark.parametrize("dataset_return_type", (dict, tuple))
@pytest.mark.parametrize("to_tensor", (transforms.ToTensor, transforms.ToImage))
def test_classif_preset(image_type, label_type, dataset_return_type, to_tensor):

    image = tv_tensors.Image(torch.randint(0, 256, size=(1, 3, 250, 250), dtype=torch.uint8))
    if image_type is PIL.Image:
        image = to_pil_image(image[0])
    elif image_type is torch.Tensor:
        image = image.as_subclass(torch.Tensor)
        assert is_pure_tensor(image)

    label = 1 if label_type is int else torch.tensor([1])

    if dataset_return_type is dict:
        sample = {
            "image": image,
            "label": label,
        }
    else:
        sample = image, label

    if to_tensor is transforms.ToTensor:
        with pytest.warns(UserWarning, match="deprecated and will be removed"):
            to_tensor = to_tensor()
    else:
        to_tensor = to_tensor()

    t = transforms.Compose(
        [
            transforms.RandomResizedCrop((224, 224), antialias=True),
            transforms.RandomHorizontalFlip(p=1),
            transforms.RandAugment(),
            transforms.TrivialAugmentWide(),
            transforms.AugMix(),
            transforms.AutoAugment(),
            to_tensor,
            # TODO: ConvertImageDtype is a pass-through on PIL images, is that
            # intended?  This results in a failure if we convert to tensor after
            # it, because the image would still be uint8 which make Normalize
            # fail.
            transforms.ConvertImageDtype(torch.float),
            transforms.Normalize(mean=[0, 0, 0], std=[1, 1, 1]),
            transforms.RandomErasing(p=1),
        ]
    )

    out = t(sample)

    assert type(out) == type(sample)

    if dataset_return_type is tuple:
        out_image, out_label = out
    else:
        assert out.keys() == sample.keys()
        out_image, out_label = out.values()

    assert out_image.shape[-2:] == (224, 224)
    assert out_label == label


@pytest.mark.parametrize("image_type", (PIL.Image, torch.Tensor, tv_tensors.Image))
@pytest.mark.parametrize("data_augmentation", ("hflip", "lsj", "multiscale", "ssd", "ssdlite"))
@pytest.mark.parametrize("to_tensor", (transforms.ToTensor, transforms.ToImage))
@pytest.mark.parametrize("sanitize", (True, False))
def test_detection_preset(image_type, data_augmentation, to_tensor, sanitize):
    torch.manual_seed(0)

    if to_tensor is transforms.ToTensor:
        with pytest.warns(UserWarning, match="deprecated and will be removed"):
            to_tensor = to_tensor()
    else:
        to_tensor = to_tensor()

    if data_augmentation == "hflip":
        t = [
            transforms.RandomHorizontalFlip(p=1),
            to_tensor,
            transforms.ConvertImageDtype(torch.float),
        ]
    elif data_augmentation == "lsj":
        t = [
            transforms.ScaleJitter(target_size=(1024, 1024), antialias=True),
            # Note: replaced FixedSizeCrop with RandomCrop, becuase we're
            # leaving FixedSizeCrop in prototype for now, and it expects Label
            # classes which we won't release yet.
            # transforms.FixedSizeCrop(
            #     size=(1024, 1024), fill=defaultdict(lambda: (123.0, 117.0, 104.0), {tv_tensors.Mask: 0})
            # ),
            transforms.RandomCrop((1024, 1024), pad_if_needed=True),
            transforms.RandomHorizontalFlip(p=1),
            to_tensor,
            transforms.ConvertImageDtype(torch.float),
        ]
    elif data_augmentation == "multiscale":
        t = [
            transforms.RandomShortestSize(
                min_size=(480, 512, 544, 576, 608, 640, 672, 704, 736, 768, 800), max_size=1333, antialias=True
            ),
            transforms.RandomHorizontalFlip(p=1),
            to_tensor,
            transforms.ConvertImageDtype(torch.float),
        ]
    elif data_augmentation == "ssd":
        t = [
            transforms.RandomPhotometricDistort(p=1),
            transforms.RandomZoomOut(fill={"others": (123.0, 117.0, 104.0), tv_tensors.Mask: 0}, p=1),
            transforms.RandomIoUCrop(),
            transforms.RandomHorizontalFlip(p=1),
            to_tensor,
            transforms.ConvertImageDtype(torch.float),
        ]
    elif data_augmentation == "ssdlite":
        t = [
            transforms.RandomIoUCrop(),
            transforms.RandomHorizontalFlip(p=1),
            to_tensor,
            transforms.ConvertImageDtype(torch.float),
        ]
    if sanitize:
        t += [transforms.SanitizeBoundingBoxes()]
    t = transforms.Compose(t)

    num_boxes = 5
    H = W = 250

    image = tv_tensors.Image(torch.randint(0, 256, size=(1, 3, H, W), dtype=torch.uint8))
    if image_type is PIL.Image:
        image = to_pil_image(image[0])
    elif image_type is torch.Tensor:
        image = image.as_subclass(torch.Tensor)
        assert is_pure_tensor(image)

    label = torch.randint(0, 10, size=(num_boxes,))

    boxes = torch.randint(0, min(H, W) // 2, size=(num_boxes, 4))
    boxes[:, 2:] += boxes[:, :2]
    boxes = boxes.clamp(min=0, max=min(H, W))
    boxes = tv_tensors.BoundingBoxes(boxes, format="XYXY", canvas_size=(H, W))

    masks = tv_tensors.Mask(torch.randint(0, 2, size=(num_boxes, H, W), dtype=torch.uint8))

    sample = {
        "image": image,
        "label": label,
        "boxes": boxes,
        "masks": masks,
    }

    out = t(sample)

    if isinstance(to_tensor, transforms.ToTensor) and image_type is not tv_tensors.Image:
        assert is_pure_tensor(out["image"])
    else:
        assert isinstance(out["image"], tv_tensors.Image)
    assert isinstance(out["label"], type(sample["label"]))

    num_boxes_expected = {
        # ssd and ssdlite contain RandomIoUCrop which may "remove" some bbox. It
        # doesn't remove them strictly speaking, it just marks some boxes as
        # degenerate and those boxes will be later removed by
        # SanitizeBoundingBoxes(), which we add to the pipelines if the sanitize
        # param is True.
        # Note that the values below are probably specific to the random seed
        # set above (which is fine).
        (True, "ssd"): 5,
        (True, "ssdlite"): 4,
    }.get((sanitize, data_augmentation), num_boxes)

    assert out["boxes"].shape[0] == out["masks"].shape[0] == out["label"].shape[0] == num_boxes_expected


@pytest.mark.parametrize("min_size", (1, 10))
@pytest.mark.parametrize("labels_getter", ("default", lambda inputs: inputs["labels"], None, lambda inputs: None))
@pytest.mark.parametrize("sample_type", (tuple, dict))
def test_sanitize_bounding_boxes(min_size, labels_getter, sample_type):

    if sample_type is tuple and not isinstance(labels_getter, str):
        # The "lambda inputs: inputs["labels"]" labels_getter used in this test
        # doesn't work if the input is a tuple.
        return

    H, W = 256, 128

    boxes_and_validity = [
        ([0, 1, 10, 1], False),  # Y1 == Y2
        ([0, 1, 0, 20], False),  # X1 == X2
        ([0, 0, min_size - 1, 10], False),  # H < min_size
        ([0, 0, 10, min_size - 1], False),  # W < min_size
        ([0, 0, 10, H + 1], False),  # Y2 > H
        ([0, 0, W + 1, 10], False),  # X2 > W
        ([-1, 1, 10, 20], False),  # any < 0
        ([0, 0, -1, 20], False),  # any < 0
        ([0, 0, -10, -1], False),  # any < 0
        ([0, 0, min_size, 10], True),  # H < min_size
        ([0, 0, 10, min_size], True),  # W < min_size
        ([0, 0, W, H], True),  # TODO: Is that actually OK?? Should it be -1?
        ([1, 1, 30, 20], True),
        ([0, 0, 10, 10], True),
        ([1, 1, 30, 20], True),
    ]

    random.shuffle(boxes_and_validity)  # For test robustness: mix order of wrong and correct cases
    boxes, is_valid_mask = zip(*boxes_and_validity)
    valid_indices = [i for (i, is_valid) in enumerate(is_valid_mask) if is_valid]

    boxes = torch.tensor(boxes)
    labels = torch.arange(boxes.shape[0])

    boxes = tv_tensors.BoundingBoxes(
        boxes,
        format=tv_tensors.BoundingBoxFormat.XYXY,
        canvas_size=(H, W),
    )

    masks = tv_tensors.Mask(torch.randint(0, 2, size=(boxes.shape[0], H, W)))
    whatever = torch.rand(10)
    input_img = torch.randint(0, 256, size=(1, 3, H, W), dtype=torch.uint8)
    sample = {
        "image": input_img,
        "labels": labels,
        "boxes": boxes,
        "whatever": whatever,
        "None": None,
        "masks": masks,
    }

    if sample_type is tuple:
        img = sample.pop("image")
        sample = (img, sample)

    out = transforms.SanitizeBoundingBoxes(min_size=min_size, labels_getter=labels_getter)(sample)

    if sample_type is tuple:
        out_image = out[0]
        out_labels = out[1]["labels"]
        out_boxes = out[1]["boxes"]
        out_masks = out[1]["masks"]
        out_whatever = out[1]["whatever"]
    else:
        out_image = out["image"]
        out_labels = out["labels"]
        out_boxes = out["boxes"]
        out_masks = out["masks"]
        out_whatever = out["whatever"]

    assert out_image is input_img
    assert out_whatever is whatever

    assert isinstance(out_boxes, tv_tensors.BoundingBoxes)
    assert isinstance(out_masks, tv_tensors.Mask)

    if labels_getter is None or (callable(labels_getter) and labels_getter({"labels": "blah"}) is None):
        assert out_labels is labels
    else:
        assert isinstance(out_labels, torch.Tensor)
        assert out_boxes.shape[0] == out_labels.shape[0] == out_masks.shape[0]
        # This works because we conveniently set labels to arange(num_boxes)
        assert out_labels.tolist() == valid_indices


def test_sanitize_bounding_boxes_no_label():
    # Non-regression test for https://github.com/pytorch/vision/issues/7878

    img = make_image()
    boxes = make_bounding_boxes()

    with pytest.raises(ValueError, match="or a two-tuple whose second item is a dict"):
        transforms.SanitizeBoundingBoxes()(img, boxes)

    out_img, out_boxes = transforms.SanitizeBoundingBoxes(labels_getter=None)(img, boxes)
    assert isinstance(out_img, tv_tensors.Image)
    assert isinstance(out_boxes, tv_tensors.BoundingBoxes)


def test_sanitize_bounding_boxes_errors():

    good_bbox = tv_tensors.BoundingBoxes(
        [[0, 0, 10, 10]],
        format=tv_tensors.BoundingBoxFormat.XYXY,
        canvas_size=(20, 20),
    )

    with pytest.raises(ValueError, match="min_size must be >= 1"):
        transforms.SanitizeBoundingBoxes(min_size=0)
    with pytest.raises(ValueError, match="labels_getter should either be 'default'"):
        transforms.SanitizeBoundingBoxes(labels_getter=12)

    with pytest.raises(ValueError, match="Could not infer where the labels are"):
        bad_labels_key = {"bbox": good_bbox, "BAD_KEY": torch.arange(good_bbox.shape[0])}
        transforms.SanitizeBoundingBoxes()(bad_labels_key)

    with pytest.raises(ValueError, match="must be a tensor"):
        not_a_tensor = {"bbox": good_bbox, "labels": torch.arange(good_bbox.shape[0]).tolist()}
        transforms.SanitizeBoundingBoxes()(not_a_tensor)

    with pytest.raises(ValueError, match="Number of boxes"):
        different_sizes = {"bbox": good_bbox, "labels": torch.arange(good_bbox.shape[0] + 3)}
        transforms.SanitizeBoundingBoxes()(different_sizes)


class TestLambda:
    inputs = pytest.mark.parametrize("input", [object(), torch.empty(()), np.empty(()), "string", 1, 0.0])

    @inputs
    def test_default(self, input):
        was_applied = False

        def was_applied_fn(input):
            nonlocal was_applied
            was_applied = True
            return input

        transform = transforms.Lambda(was_applied_fn)

        transform(input)

        assert was_applied

    @inputs
    def test_with_types(self, input):
        was_applied = False

        def was_applied_fn(input):
            nonlocal was_applied
            was_applied = True
            return input

        types = (torch.Tensor, np.ndarray)
        transform = transforms.Lambda(was_applied_fn, *types)

        transform(input)

        assert was_applied is isinstance(input, types)
