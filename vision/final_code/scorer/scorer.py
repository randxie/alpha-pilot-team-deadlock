from shapely.geometry import Polygon
import numpy as np

class mAPScorer():

    def __init__(self):
        self.THRESHOLD = .5


    def create_poly(self, box):
        x1, y1, x2, y2, x3, y3, x4, y4 = box
        return [(x1, y1), (x2, y2), (x3, y3),
                (x4, y4), (x1, y1)]

    def calculate_iou(self, polygon1, polygon2):
        try:
            polygon1 = Polygon(polygon1)
            polygon2 = Polygon(polygon2)
            polygon1 = polygon1.buffer(0)
            polygon2 = polygon2.buffer(0)
            intersection = polygon1.intersection(polygon2)
            union = polygon1.area + polygon2.area - intersection.area
            return intersection.area / union
        except:
            return 0.0

    def Pascal_mAP(self, GT_data, pred_data):
        tp_fp_cf = []
        n_GT = self.countBoxes(GT_data)
        for img_key in list(GT_data.keys()):
            # get the ground truth test_box
            GT_box = GT_data[img_key]
            # get prediction boxes stored as Nx9
            # np array x1,y1,....,x4,y4, confidence
            if img_key in pred_data.keys():
                pred_box = pred_data[img_key]
            else:
                pred_box = []
            #if a ground predicitons are present:
            if (len(pred_box) > 0):
                tp_fp_cf_i = self.metrics(
                                          np.array(GT_box),
                                          np.array(pred_box), self.THRESHOLD)
                tp_fp_cf += tp_fp_cf_i
        mAP, mAP_points = self.map_eleven(tp_fp_cf, n_GT)
        return mAP, mAP_points

    def COCO_mAP(self, GT_data, pred_data):
        IOU_all = np.linspace(0.05, 0.95, 10)
        sumAP = 0
        for iou_threshold in IOU_all:
            sumAP += self.IOU_mAP(GT_data, pred_data, iou_threshold)
        mAP = sumAP / len(IOU_all)
        return mAP

    def IOU_mAP(self, GT_data, pred_data, IOU_threshold):
        tp_fp_cf = []
        n_GT = self.countBoxes(GT_data)
        for img_key in list(GT_data.keys()):
            # get the ground truth test_box
            GT_box = GT_data[img_key]
            # get prediction boxes stored as
            # Nx9 np array x1,y1,....,x4,y4, confidence
            if img_key in pred_data.keys():
                pred_box = pred_data[img_key]
            else:
                pred_box = [[]]
            # if a ground truth box is presen

            if (len(pred_box) > 0) :
                tp_fp_cf_i = self.metrics(
                                          np.array(GT_box),
                                          np.array(pred_box),
                                          IOU_threshold)
                tp_fp_cf += tp_fp_cf_i
        mAP, mAP_points = self.map_eleven(np.array(tp_fp_cf), n_GT)
        return mAP

    def countBoxes(self, dict_):
        n = 0
        for key in list(dict_.keys()):
            n += len(dict_[key])
        return n

    def metrics(self, truth_boxes, test_boxes, IOU_threshold):
        # all should be numpy arrays
        # truth box is 1x8: [x1,y1...x4, y4]
        # test_boxes is Nx9: [x1,y1... x4, y4, confidence]
        # n is the number of predictions for a given image
        ious = []
        n, _ = test_boxes.shape
        tp_fp_cf = np.zeros(shape=(n, 3))
        m, n = truth_boxes.shape
        if n == 0:
            tp_fp_cf[:, 1] = 1
            tp_fp_cf[:, 2] = np.array(test_boxes)[:, -1]
            return tp_fp_cf.tolist()
        # tp, fp, confidence
        # assign all boxes as false positives
        # Test which boxes detect given GT box.
        tp_fp_cf[:, 1] = 1
        tp_fp_cf[:, 2] = np.array(test_boxes)[:, -1]
        for truth_box in truth_boxes:
            # for each ground truth box,
            # compute max IOU, and check if maxIOU>threshold,
            # assign the corresponding box to gt box
            truth_poly = self.create_poly(truth_box)
            box_id = 0
            for test_box in test_boxes:
                # if box is already assigned,
                # tp_fp_cf[box_id,1]=0, so dont use it
                iou_box = self.calculate_iou(
                                             truth_poly,
                                             self.create_poly(test_box[0: 8]))
                ious.append(tp_fp_cf[box_id, 1] * iou_box)
                box_id += 1
            max_iou = np.max(ious)
            argmax_iou = np.argmax(ious)
            if (max_iou > IOU_threshold):
                tp_fp_cf[argmax_iou, 0] = 1
                tp_fp_cf[argmax_iou, 1] = 0
        return tp_fp_cf.tolist()

    def map_eleven(self, tp_fp_cf, total_truths):
        # tp fp cf is a Nx3 array
        # N is the number of box predications
        tp_fp_cf = np.array(sorted(
                                   tp_fp_cf,
                                   key=lambda x: x[-1],
                                   reverse=True))
        tp = np.cumsum(tp_fp_cf[:, 0])
        fp = np.cumsum(tp_fp_cf[:, 1])
        rec = tp / total_truths
        prec = np.divide(tp, (fp + tp))
        recalls_thresholds = np.linspace(0, 1, 11)
        recalls_thresholds[-1] = 0.9999
        map_ = 0
        map_all = []
        for thresh in recalls_thresholds:
            indexs = np.where(rec >= thresh)
            if len(indexs[0]) > 0:
                prcn = np.max(prec[indexs])
            else:
                prcn = 0
            map_ += prcn
            map_all.append(prcn)
        return map_ / 11., map_all
