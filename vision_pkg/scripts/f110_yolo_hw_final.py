# -*- coding: utf-8 -*-

from pathlib import Path
from PIL import Image
import numpy as np
from copy import deepcopy
import matplotlib as plt
import matplotlib.patches as patches
from typing import Tuple
import time

import pycuda.autoinit
import pycuda.driver as cuda
import tensorrt as trt
import torch



def allocate_buffers(engine):
    for binding in engine:
        if engine.binding_is_input(binding):  # we expect only one input
            input_shape = engine.get_binding_shape(binding)
            input_size = trt.volume(input_shape) * engine.max_batch_size * np.dtype(np.float32).itemsize  # in bytes
            input_dtype = trt.nptype(engine.get_tensor_dtype(binding))
            host_input = cuda.pagelocked_empty(input_size, input_dtype)
            device_input = cuda.mem_alloc(input_size)
        else:  # and one output
            output_shape = engine.get_binding_shape(binding)
            # create page-locked memory buffers (i.e. won't be swapped to disk)
            host_output = cuda.pagelocked_empty(trt.volume(output_shape) * engine.max_batch_size, dtype=np.float32)
            device_output = cuda.mem_alloc(host_output.nbytes)
    return host_input, device_input, host_output, device_output

def DisplayLabel(image, bboxs):
    # image = np.transpose(image.copy(), (1, 2, 0))
    # fig, ax = plt.subplots(1, figsize=(6, 8))
    # image = cv.cvtColor(image, cv.COLOR_BGR2RGB)
    fig, ax = plt.subplots(1)
    edgecolor = [1, 0, 0]
    if len(bboxs) == 1:
        bbox = bboxs[0]
        ax.add_patch(patches.Rectangle((bbox[0] - bbox[2]/2, bbox[1] - bbox[3]/2), bbox[2], bbox[3], linewidth=1, edgecolor=edgecolor, facecolor='none'))
        print(f"One bounding box found at {bbox}")
    elif len(bboxs) > 1:
        for bbox in bboxs:
            ax.add_patch(patches.Rectangle((bbox[0] - bbox[2]/2, bbox[1] - bbox[3]/2), bbox[2], bbox[3], linewidth=1, edgecolor=edgecolor, facecolor='none'))
    # print(f"Bounding boxes predicted: {bboxs}")
    ax.imshow(image)
    plt.savefig("detection_output.png")
    plt.show()

# convert from [c_x, c_y, w, h] to [x_l, y_l, x_r, y_r]
def bbox_convert(c_x, c_y, w, h):
    return [c_x - w/2, c_y - h/2, c_x + w/2, c_y + h/2]

# convert from [x_l, y_l, x_r, x_r] to [c_x, c_y, w, h]
def bbox_convert_r(x_l, y_l, x_r, y_r):
    return [x_l/2 + x_r/2, y_l/2 + y_r/2, x_r - x_l, y_r - y_l]

# calculating IoU
def IoU(a, b):
    # referring to IoU algorithm in slides
    inter_w = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    inter_h = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    inter_ab = inter_w * inter_h
    area_a = (a[3] - a[1]) * (a[2] - a[0])
    area_b = (b[3] - b[1]) * (b[2] - b[0])
    union_ab = area_a + area_b - inter_ab
    if union_ab == 0:
        return 0
    return inter_ab / union_ab

def grid_cell(cell_indx, cell_indy, anchor_size):
    stride_0 = anchor_size[1]
    stride_1 = anchor_size[0]
    return np.array([cell_indx * stride_0, cell_indy * stride_1, cell_indx * stride_0 + stride_0, cell_indy * stride_1 + stride_1])

def label_to_box_xyxy(result: torch.Tensor,
                      input_shape: Tuple,
                      output_shape: Tuple,
                      anchor_size: tuple,
                      threshold = 0.9):
    validation_result = []
    result_prob = []
    for ind_row in range(output_shape[2]):
        for ind_col in range(output_shape[3]):
            grid_info = grid_cell(ind_col, ind_row, anchor_size)
            validation_result_cell = []
            if result[0, ind_row, ind_col] >= threshold:
                c_x = grid_info[0] + anchor_size[1]/2 + result[1, ind_row, ind_col]
                c_y = grid_info[1] + anchor_size[0]/2 + result[2, ind_row, ind_col]
                w = result[3, ind_row, ind_col] * input_shape[2]
                h = result[4, ind_row, ind_col] * input_shape[3]
                x1, y1, x2, y2 = bbox_convert(c_x, c_y, w, h)
                x1 = np.clip(x1, 0, input_shape[3])
                x2 = np.clip(x2, 0, input_shape[3])
                y1 = np.clip(y1, 0, input_shape[2])
                y2 = np.clip(y2, 0, input_shape[2])
                validation_result_cell.append(x1)
                validation_result_cell.append(y1)
                validation_result_cell.append(x2)
                validation_result_cell.append(y2)
                result_prob.append(result[0, ind_row, ind_col])
                validation_result.append(validation_result_cell)
    validation_result = np.array(validation_result)
    result_prob = np.array(result_prob)
    return validation_result, result_prob

def voting_suppression(result_box, iou_threshold = 0.5):
    votes = np.zeros(result_box.shape[0])
    for ind, box in enumerate(result_box):
        for box_validation in result_box:
            if IoU(box_validation, box) > iou_threshold:
                votes[ind] += 1
    return (-votes).argsort()


def run_inference():

    # 1. PREPROCESSING
    trt_model_path = Path(r"/home/tanaya/lab-7-vision-lab-team-10/yolof110_updated.trt")
    test_image_path = Path(r"/home/tanaya/lab-7-vision-lab-team-10/resource/test_car_x60cm.png")
    # Expected output shape.

    output_shape = (1,5,5,10)

    # Model input shape (b, c, h, w)
    model_input_shape = (1, 3, 180, 320)

    # Compute the size of the anchor boxes.
    anchor_size = [(model_input_shape[2] / output_shape[2]), (model_input_shape[3] / output_shape[3])]
    # Take the model input shape and convert it to the format Pillow accepts
    # (Width, Height). We can tell Pillow to resize
    resize_shape_wh = (model_input_shape[3], model_input_shape[2])
    test_image = Image.open(test_image_path)
    raw_image_shape = test_image.size
    test_image = test_image.resize(resize_shape_wh, resample=Image.Resampling.BICUBIC)
    test_image = np.array(test_image, dtype=np.float32, order="C")
    # Normalize the test image.
    test_image = test_image / 255.0

    # Image should be in H,W,C
    numpy_image = deepcopy(test_image)
    # cv.imwrite("temp_img.png", numpy_image)

    # Reshape the np array that used to be a PIL image into "CHW" (PyTorch shape
    # convention).
    test_image = np.transpose(test_image, [2, 0, 1])
    # CHW to NCHW format. I.e., adding another dimension in case you had a batch
    # size greater than 1.
    test_image = np.expand_dims(test_image, axis=0)
    # Convert the image to row major order in memory.
    test_image = np.array(test_image, dtype=np.float32, order="C")


    # 2. INFERENCE
    # Set up runtime and deserialize the engine file (the TensorRT model
    f = open(trt_model_path, "rb")
    runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))

    f1yolo_engine = runtime.deserialize_cuda_engine(f.read())
    context = f1yolo_engine.create_execution_context()

    # Allocate memory for input and output.
    host_input, device_input, host_output, device_output = allocate_buffers(engine=f1yolo_engine)
    # host_input is a region of page_locked memory on the host, while
    # device_input is a memory location allocated on the GPU. Likewise,
    # host_output is a region of page_locked memory on the host, and
    # device_output is the memory region on the device that is allocated to
    # store the output of the engine.

    # Also, we create what's called a CUDA stream object. A CUDA stream is kind
    # of like an OS thread--it's an independent sequence of (CUDA) instructions.
    # For our case, we only need one stream, as we only want the capability to
    # process a single image at a time. BUT, you could create multiple, parallel
    # streams to process multiple images asynchronously if you needed to. To add
    # instructions to this stream, we can specify this stream as an argument to
    # each of the CUDA operations that we specify below.
    inference_stream = cuda.Stream()

    # Pass image through model.
    print(f"Running inference on {test_image_path}")
    # Copy the input image into the host_input page_locked memory.
    host_input = test_image
    # Copy the image data in the hosts memory to the device's memory.
    inference_start = time.perf_counter()
    cuda.memcpy_htod_async(device_input, host_input, inference_stream)
    # Run inference. This essentially tells teh CPU to take the input and pass
    # it through the engine, place the result in the output location.
    context.execute_async(bindings=[int(device_input), int(device_output)],
                          stream_handle=inference_stream.handle)
    # Copy the results from GPU memory back to the host.
    cuda.memcpy_dtoh_async(host_output, device_output, inference_stream)
    inference_stream.synchronize()
    inference_end = time.perf_counter()
    print(f"Inference performed in {(inference_end - inference_start)*1000:.2f}ms.")

    # The output tensors will be flat arrays out of the engine, so need to
    # reshape these to the expected output format to make sense of them.
    result_tensor = torch.Tensor(host_output)
    print(f"Result vector has shape: {result_tensor.shape}")
    # Reshape result tensor.
    result_tensor = result_tensor.reshape(output_shape)
    print(f"Reshaped output vector shape: {result_tensor.shape}")

    # 3. POST PROCESSING.
    # Now, begin the "post processing." For this, we basically want to run NMS
    # on the predicted output bounding boxes (each of the 6x10==60 5-element
    # predicted bboxes) and return those bounding boxes above the specified
    # confidence threshold.

    # Then, as a debugging step, can draw those bounding boxes on the image.

    # UPDATE: The notebook already provides functions for working with this
    # particular model's outputs. So, once we reshape above, we should be able
    # to use the existing functions to get things into the proper form.

    # display detection
    voting_iou_threshold = 0.5
    confi_threshold = 0.4

    # result = model(image_t)
    # result = result.detach().cpu().numpy()
    # NOTE Need to know what the output dimensions are. I.e., what shape is
    # result here?
    bboxes, result_prob = label_to_box_xyxy(result=result_tensor[0],
                                           input_shape=model_input_shape,
                                           output_shape=output_shape,
                                           anchor_size=anchor_size)
    vote_rank = voting_suppression(bboxes, voting_iou_threshold)
    bbox = bboxes[vote_rank[0]]
    [c_x, c_y, w, h] = bbox_convert_r(bbox[0], bbox[1], bbox[2], bbox[3])
    bboxs_2 = np.array([[c_x, c_y, h, w]])
    # DisplayLabel(np.transpose(test_image[0], (1, 2, 0)), bboxs_2)
    DisplayLabel(image=numpy_image, bboxs=bboxs_2)
    print(f"Confidence: {result_prob[vote_rank[0]]*100:.2f}%")

    return c_x, c_y, h, w, result_prob[vote_rank[0]]


# if __name__ == "__main__":

#     main()