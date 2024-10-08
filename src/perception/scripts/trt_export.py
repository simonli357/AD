from ultralytics import YOLO
import tensorrt as trt
# from ultralytics import YOLO
import onnx
import lib.common as common
import os
pt_path = os.path.dirname(os.path.realpath(__file__)).replace("scripts", "models/trt/citycocov2lgtclab_20.pt")
model = YOLO(pt_path)  # initialize

model.export(format='engine')  # export

# Path to the ONNX model file
# onnx_model_path = os.path.dirname(os.path.realpath(__file__)).replace("scripts", "models/yolov8n.onnx")

# # Load the ONNX model
# onnx_model = onnx.load(onnx_model_path)

# # Create a TensorRT engine
# TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
# with trt.Builder(TRT_LOGGER) as builder, builder.create_network(
#     common.EXPLICIT_BATCH
# ) as network, builder.create_builder_config() as config, trt.OnnxParser(
#     network, TRT_LOGGER
# ) as parser, trt.Runtime(
#     TRT_LOGGER
# ) as runtime:
#     config.max_workspace_size = 1 << 28  # 256MiB
#     builder.max_batch_size = 1
#     engine = None

#     if not parser.parse(onnx_model.SerializeToString()):
#         print("ERROR: Failed to parse the ONNX file.")
#     else:
#         plan = builder.build_serialized_network(network,config)
#         # engine = runtime.deserialize_cuda_engine(plan)
#         with open(os.path.dirname(os.path.realpath(__file__)).replace("scripts", "models/yolov8n.engine"), 'wb') as f:
#             f.write(plan)
