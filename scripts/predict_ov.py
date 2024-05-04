from ultralytics.data.augment import LetterBox
import cv2
import numpy as np
import openvino.runtime as ov
from typing import Tuple
from ultralytics.utils import ops
import torch


def postprocess(
    pred_boxes: np.ndarray,
    input_hw: Tuple[int, int],
    orig_img: np.ndarray,
    min_conf_threshold: float = 0.25,
    nms_iou_threshold: float = 0.7,
    agnosting_nms: bool = False,
    max_detections: int = 300,
):
    """
    YOLOv8 model postprocessing function. Applied non maximum supression algorithm to detections and rescale boxes to original image size
    Parameters:
        pred_boxes (np.ndarray): model output prediction boxes
        input_hw (np.ndarray): preprocessed image
        orig_image (np.ndarray): image before preprocessing
        min_conf_threshold (float, *optional*, 0.25): minimal accepted confidence for object filtering
        nms_iou_threshold (float, *optional*, 0.45): minimal overlap score for removing objects duplicates in NMS
        agnostic_nms (bool, *optiona*, False): apply class agnostinc NMS approach or not
        max_detections (int, *optional*, 300):  maximum detections after NMS
    Returns:
       pred (List[Dict[str, np.ndarray]]): list of dictionary with det - detected boxes in format [x1, y1, x2, y2, score, label] and 
                                           kpt - 17 keypoints in format [x1, y1, score1]
    """
    nms_kwargs = {"agnostic": agnosting_nms, "max_det": max_detections}
    print(torch.from_numpy(pred_boxes).shape)
    preds = ops.non_max_suppression(
        torch.from_numpy(pred_boxes),
        min_conf_threshold,
        nms_iou_threshold,
        nc=4,
        **nms_kwargs
    )
    results = []

    kpt_shape = [4, 2]
    for i, pred in enumerate(preds):
        shape = orig_img[i].shape if isinstance(
            orig_img, list) else orig_img.shape
        box = pred[:, :6].numpy()
        pred_kpts = pred[:, -8:].numpy()
        results.append({"box": box, 'kpt': pred_kpts})

    return results


def preprocess(im, imsz=(640, 640)):
    """
    Prepares input image before inference.

    Args:
        im (torch.Tensor | List(np.ndarray)): BCHW for tensor, [(HWC) x B] for list.
    """
    not_tensor = not isinstance(im, torch.Tensor)
    if not_tensor:
        im = np.stack(pre_transform(im, imsz))
        print("Before preprocessing, shape:", im.shape)
        # BGR to RGB, BHWC to BCHW, (n, 3, h, w)
        im = im[..., ::-1].transpose((0, 3, 1, 2))
        im = np.ascontiguousarray(im)  # contiguous
        im = torch.from_numpy(im)

    im = im.to('cpu')
    im = im.float()  # uint8 to fp16/32
    if not_tensor:
        im /= 255  # 0 - 255 to 0.0 - 1.0
    return im


def pre_transform(im, imgsz=(640, 640)):
    """
    Pre-transform input image before inference.

    Args:
        im (List(np.ndarray)): (N, 3, h, w) for tensor, [(h, w, 3) x N] for list.

    Returns:
        (list): A list of transformed images.
    """
    same_shapes = all(x.shape == im[0].shape for x in im)
    letterbox = LetterBox(imgsz, auto=same_shapes and True, stride=32)
    return [letterbox(image=x) for x in im]


# 加载YOLOv8模型
# model_path = './models/armor-5_quantized.xml'
model_path = 'models/armor-5.xml'
img_path = './input/sample.jpg'
ori_img = cv2.imread(img_path)
img = cv2.resize(ori_img, (640, 640))
# img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
img_data = preprocess([img])
data = img_data.numpy()

# data = np.transpose(img, (2, 0, 1))
# data = np.expand_dims(data, axis=0)
# print(data.shape)


core = ov.Core()
model = core.read_model(model_path)
compiled_model = core.compile_model(model, 'CPU')
infer_request = compiled_model.create_infer_request()

input_tensor = infer_request.get_input_tensor(0)
print(input_tensor.data.shape, data.shape)
print(type(input_tensor.data), type(data))
input_tensor.data[...] = data
print(input_tensor)

infer_request.infer()

output_tensor = infer_request.get_output_tensor(0)
print(output_tensor.data.shape)
result = postprocess(output_tensor.data, (640, 640), img)
print(result)

res_img = img.copy()
# res_img = cv2.cvtColor(res_img, cv2.COLOR_RGB2BGR)
for box in result[0]['box']:
    box = box.astype(np.int32)
    cv2.putText(res_img, str(box[4]), (int(box[0]), int(
        box[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
    cv2.rectangle(res_img, (int(box[0]), int(box[1])), (int(
        box[2]), int(box[3])), (0, 0, 255), 2)
for kpt in result[0]['kpt']:
    for x, y in zip(kpt[::2], kpt[1::2]):
        cv2.circle(res_img, (int(x), int(y)), 3, (0, 255, 0), -1)

res_img = cv2.resize(res_img, (ori_img.shape[1], ori_img.shape[0]))
cv2.imshow('res_img', res_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
