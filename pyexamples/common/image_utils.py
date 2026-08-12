# mostly from https://github.com/cvg/LightGlue
import cv2, os, torch
import numpy as np

def read_image(path, grayscale: bool = False) -> np.ndarray:
    """Read an image from path as RGB or grayscale"""
    if not os.path.exists(path):
        raise FileNotFoundError(f'No image at path {path}.')
    mode = cv2.IMREAD_GRAYSCALE if grayscale else cv2.IMREAD_COLOR
    image = cv2.imread(str(path), mode)
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    if image is None:
        raise IOError(f'Could not read image at {path}.')
    if not grayscale:
        image = image[..., ::-1]
    return image


def numpy_image_to_torch(image: np.ndarray) -> torch.Tensor:
    """Normalize the image tensor and reorder the dimensions."""
    if image.ndim == 3:
        image = image.transpose((2, 0, 1))  # HxWxC to CxHxW
    elif image.ndim == 2:
        image = image[None]  # add channel axis
    else:
        raise ValueError(f'Not an image: {image.shape}')
    return torch.tensor(image / 255., dtype=torch.float)


def resize_image(image, size, fn = "max", interp = 'area'):
    """Resize an image to a fixed size, or according to max or min edge."""
    h, w = image.shape[:2]

    fn = {'max': max, 'min': min}[fn]
    if isinstance(size, int):
        scale = size / fn(h, w)
        h_new, w_new = int(round(h*scale)), int(round(w*scale))
        scale = (w_new / w, h_new / h)
    elif isinstance(size, (tuple, list)):
        h_new, w_new = size
        scale = (w_new / w, h_new / h)
    else:
        raise ValueError(f'Incorrect new size: {size}')
    mode = {
        'linear': cv2.INTER_LINEAR,
        'cubic': cv2.INTER_CUBIC,
        'nearest': cv2.INTER_NEAREST,
        'area': cv2.INTER_AREA}[interp]
    return cv2.resize(image, (w_new, h_new), interpolation=mode), scale


def load_image(path, resize = None, **kwargs) -> torch.Tensor:
    image = read_image(path)
    scale = (1.,1.)
    if resize is not None:
        image, scale = resize_image(image, resize, **kwargs)
    return numpy_image_to_torch(image), torch.tensor(scale)