import numpy as np
import matplotlib.pyplot as plt


def plot_bbox(img, bbox, color='c'):
  plt.imshow(img)

  for bb_i in bbox:
    x1 = bb_i[0]
    y1 = bb_i[1]
    x2 = bb_i[2]
    y2 = bb_i[3]
    x3 = bb_i[4]
    y3 = bb_i[5]
    x4 = bb_i[6]
    y4 = bb_i[7]
    plt.plot([x1, x2, x3, x4, x1],
             [y1, y2, y3, y4, y1],
             color=color, linewidth=3
             )
  plt.axis('off')


def plot_GT_pred(img, bbox, lbl, bbox_gt, color='c', color_gt='g'):
  plt.imshow(img)
  for i_ in range(len(bbox)):
    bb_i = bbox[i_]
    lbl_i = lbl[i_]
    x1 = bb_i[0]
    y1 = bb_i[1]
    x2 = bb_i[2]
    y2 = bb_i[3]
    x3 = bb_i[4]
    y3 = bb_i[5]
    x4 = bb_i[6]
    y4 = bb_i[7]
    plt.plot([x1, x2, x3, x4, x1],
             [y1, y2, y3, y4, y1],
             color=color, linewidth=3
             )
    str_test = 'p = ' + str(round(lbl_i[2], 3))
    plt.text(x2, y2, str_test, color=color)
  for bb_i in bbox_gt:
    x1 = bb_i[0]
    y1 = bb_i[1]
    x2 = bb_i[2]
    y2 = bb_i[3]
    x3 = bb_i[4]
    y3 = bb_i[5]
    x4 = bb_i[6]
    y4 = bb_i[7]
    plt.plot([x1, x2, x3, x4, x1],
             [y1, y2, y3, y4, y1],
             color=color_gt, linewidth=2
             )
  plt.axis('off')
