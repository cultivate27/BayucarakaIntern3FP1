import torch
import rospy
from geometry_msgs.msg import Point
import cv2
import numpy as np
from yolov5.models.common import DetectMultiBackend
from yolov5.utils.general import non_max_suppression
from yolov5.utils.torch_utils import select_device

class DetectNode:
    def __init__(self):
        rospy.init_node('detect_node', anonymous=True)
        self.publisher = rospy.Publisher('/detected_object_coordinates', Point, queue_size=10)
        self.device = select_device('')
        self.model = DetectMultiBackend('models/exp4/weights/best.pt', device=self.device)
        self.model.warmup()

    def process_video(self):
        cap = cv2.VideoCapture(0)

        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                break

            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img = np.transpose(img, (2, 0, 1))
            img = torch.from_numpy(img).to(self.device).float() / 255.0

            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            preds = self.model(img)
            preds = non_max_suppression(preds)[0]
            
            for det in preds:
                x_min, y_min, x_max, y_max, conf, cls = det[:6]

                x_center = (x_min + x_max) / 2
                y_center = (y_min + y_max) / 2

                point_msg = Point()
                point_msg.x = float(x_center)
                self.publisher.publish(point_msg)

                width = x_max - x_min
                height = y_max - y_min
                print(f"Koordinat Objek: ({x_center}, {y_center}, 2.0), Width: {width}, Height: {height}")

                color = (0, 0, 255)
                thickness = 2 
                start_point = (int(x_min), int(y_min))
                end_point = (int(x_max), int(y_max))
                cv2.rectangle(frame, start_point, end_point, color, thickness)
                
                label = f"Class: {int(cls)}, Conf: {conf:.2f}"
                cv2.putText(frame, label, (int(x_min), int(y_min) - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # DEBUG dalam bentuk video
            cv2.imshow('Deteksi Objek', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    node = DetectNode()
    try:
        node.process_video()
    except rospy.ROSInterruptException:
        pass
