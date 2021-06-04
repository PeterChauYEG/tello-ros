from threading import Thread
import numpy as np
import cv2
from tellobot.ai_constants import OBJECT_DETECTION_MODEL_FILE, OBJECT_DETECTION_CONFIG_FILE, OBJECT_DETECTION_CLASSES_FILE


class ObjectDetection:
  def __init__(self):
    self.prob_threshold = 0.05

    self.input_w = 128
    self.input_h = 128

    self.frame_w = None
    self.frame_h = None

    self.objects = None
    self.object_labels = None
    self.thread_started = False
    self.current_frame = None
    self.detections = None

    # init ===============
    # read the neural network of the pose recognition
    self.net = cv2.dnn.readNetFromTensorflow(OBJECT_DETECTION_MODEL_FILE, OBJECT_DETECTION_CONFIG_FILE)
    self.classes = np.loadtxt(OBJECT_DETECTION_CLASSES_FILE, dtype=np.str, delimiter='\n')
    self.thread = Thread(target=self.update, args=(), daemon=True)
    self.start()

  def set_current_frame(self, frame):
    self.current_frame = frame

  def start(self):
    self.thread_started = True
    self.thread.start()

  def update(self):
    while True:
      if not self.thread_started:
        return

      if self.current_frame is not None:
        self.detect()
        self.current_frame = None

  def read(self):
    return self.objects, self.object_labels

  def stop(self):
    self.thread_started = False

  def preprocess(self, frame):
    frame = cv2.bilateralFilter(frame, 5, 50, 100)

    if self.frame_w is None or self.frame_h is None:
      self.frame_w = frame.shape[1]
      self.frame_h = frame.shape[0]

    frame_blob = cv2.dnn.blobFromImage(frame, 1.0 / 255, (self.input_w, self.input_h),
                                       (0, 0, 0), swapRB=False, crop=False)
    self.net.setInput(frame_blob)

    return frame

  def clear_detection_state(self):
    self.objects = None
    self.detections = None
    self.object_labels = None

  def postprocess(self):
    detections = self.detections[0][0]
    self.objects = []
    self.object_labels = []

    for detection in detections:
      # confidence
      if float(detection[2]) > 0.3:
        processed_detection = [
          int(detection[3] * self.frame_w),
          int(detection[4] * self.frame_h),
          int(detection[5] * self.frame_w),
          int(detection[6] * self.frame_w)
        ]
        processed_label = self.classes[int(detection[0])]
        
        self.objects.append(processed_detection)
        self.object_labels.append(processed_label)

  def detect(self):
    self.clear_detection_state()

    self.preprocess(self.current_frame)

    self.detections = self.net.forward()
    self.postprocess()
