import ncnn
import numpy as np
import math

class FaceInfo:
    def __init__(self, x1=0,y1=0,x2=0,y2=0,score=0):
        self.x1 = x1
        self.y1 = y1
        self.x2 = x2
        self.y2 = y2
        self.score = score

class UltraFaceNcnn:
    def __init__(self, param, model, input_size=(320,240),
                 threshold=0.7, nms_threshold=0.3, num_thread=1):
        self.net = ncnn.Net()
        self.net.opt.use_vulkan_compute = False
        self.net.opt.num_threads = num_thread
        self.net.load_param(param)
        self.net.load_model(model)

        self.in_w, self.in_h = input_size
        self.threshold = threshold
        self.nms_threshold = nms_threshold

        self.center_variance = 0.1
        self.size_variance = 0.2

        self.min_boxes = [[10,16,24],[32,48],[64,96],[128,192,256]]
        self.strides = [8,16,32,64]

        self.priors = self._generate_priors()

    def _generate_priors(self):
        priors = []
        for stride, min_box in zip(self.strides, self.min_boxes):
            fm_w = math.ceil(self.in_w / stride)
            fm_h = math.ceil(self.in_h / stride)
            for i in range(fm_h):
                for j in range(fm_w):
                    cx = (j+0.5)*stride/self.in_w
                    cy = (i+0.5)*stride/self.in_h
                    for m in min_box:
                        w = m/self.in_w
                        h = m/self.in_h
                        priors.append([cx, cy, w, h])
        return np.array(priors, dtype=np.float32)

    @staticmethod
    def _clip(x, y):
        return max(0.0, min(x, y))

    def _decode_boxes(self, boxes):
        priors = self.priors
        cx = boxes[:,0]*self.center_variance*priors[:,2] + priors[:,0]
        cy = boxes[:,1]*self.center_variance*priors[:,3] + priors[:,1]
        w  = np.exp(boxes[:,2]*self.size_variance) * priors[:,2]
        h  = np.exp(boxes[:,3]*self.size_variance) * priors[:,3]

        x1 = cx - w/2
        y1 = cy - h/2
        x2 = cx + w/2
        y2 = cy + h/2
        return np.stack([x1,y1,x2,y2], axis=1)

    def _nms(self, boxes, scores):
        idx = scores.argsort()[::-1]
        keep = []
        while len(idx) > 0:
            i = idx[0]
            keep.append(i)
            if len(idx) == 1: break
            xx1 = np.maximum(boxes[i,0], boxes[idx[1:],0])
            yy1 = np.maximum(boxes[i,1], boxes[idx[1:],1])
            xx2 = np.minimum(boxes[i,2], boxes[idx[1:],2])
            yy2 = np.minimum(boxes[i,3], boxes[idx[1:],3])

            w = np.maximum(0, xx2-xx1)
            h = np.maximum(0, yy2-yy1)
            inter = w*h
            area1 = (boxes[i,2]-boxes[i,0])*(boxes[i,3]-boxes[i,1])
            area2 = (boxes[idx[1:],2]-boxes[idx[1:],0])*(boxes[idx[1:],3]-boxes[idx[1:],1])
            iou = inter / (area1+area2-inter)
            idx = idx[np.where(iou <= self.nms_threshold)[0]+1]
        return keep

    def detect(self, img):
        h, w = img.shape[:2]

        mat_in = ncnn.Mat.from_pixels_resize(img, ncnn.Mat.PixelType.PIXEL_BGR,
                                             img.shape[1], img.shape[0],
                                             self.in_w, self.in_h)
        mat_in.substract_mean_normalize((127,127,127),(1.0/128,1.0/128,1.0/128))

        ex = self.net.create_extractor()
        ex.input("input", mat_in)

        mat_scores = ncnn.Mat()
        mat_boxes = ncnn.Mat()
        ex.extract("scores", mat_scores)
        ex.extract("boxes", mat_boxes)

        boxes = np.array(mat_boxes)
        scores = np.array(mat_scores)[:,1]  # face class

        boxes = self._decode_boxes(boxes)

        # threshold后再筛选
        mask = scores >= self.threshold
        boxes = boxes[mask]
        scores = scores[mask]
        if len(boxes)==0:
            return []

        keep = self._nms(boxes, scores)

        results = []
        for i in keep:
            x1,y1,x2,y2 = boxes[i]
            results.append(FaceInfo(
                self._clip(x1,1)*w,
                self._clip(y1,1)*h,
                self._clip(x2,1)*w,
                self._clip(y2,1)*h,
                float(scores[i])
            ))
        return results

