# Note Detection With YOLOv8 On Orangepi5 RK3588 NPU

This guide is only designed for the orangepi5 and rk3588 npu, however it is possible to use with other chips by modifying the code.

# Pipeline

Download dataset from roboflow -> upload dataset to gdrive -> train yolov8 model using gpu hardware accelerator with google colab -> convert .pt to .onnx model -> convert .onnx to .rknn model-> copy .rknn file to opi5 -> get rknn toolkit2 again on the opi5 for the runtime -> get images from video stream -> make a python file to use RKNN api and run it on an image, e.g. https://github.com/cluangar/YOLOv5-RK3588-Python/tree/singlethread -> send note location data to roborio using networktables

# Training the Model

1. Download the note data from [roboflow](https://universe.roboflow.com/note-detection-frc/note-detection-frc-2024 "roboflow dataset") and select YOLOv8 annotation option
2. Upload the data to google drive
3. Use google colab to train the data using YOLOv8
   1. Under hardware accelerators, enable T4 GPU
   2. The number of epochs should be chosen such that the learning curve begins to level off, around 25 epochs works well
   3. Google offers 12 hours of continous runtime, but only 90 minutes if the browser is closed due to inactivity.
4. Download the best.pt file under runs/detect

# Converting the Model

As running inference using just the cpu on the orangepi is very slow(500ms inference with ultralytics lib), we need to convert the model to an .rknn model in order to be ran on the neural processing unit(npu) of the rk3588.

## Converting .pt to .onnx

Using the yolov8 library to export as .onnx does not work. Instead, we must use [this](https://github.com/airockchip/ultralytics_yolov8/blob/main/ "airockchip yolov8 fork") fork of the yolov8 library to convert to .onnx. Clone the repo and follow the instructions in the README to convert to .onnx.

## Converting .onnx to .rknn

In order to convert to .rknn, we must use the rknn toolkit 2. In a linux environment, clone the [toolkit](https://github.com/airockchip/rknn-toolkit2/tree/master "rknn-toolkit-2"). Note that this does not work on windows, and it is possible for this step to be completed on the orangepi. Clone the [rknn-model-zoo](https://github.com/airockchip/rknn_model_zoo "rknn-model-zoo") and run the convert.py file under yolov8.
