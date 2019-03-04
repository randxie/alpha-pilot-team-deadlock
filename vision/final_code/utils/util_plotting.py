import numpy as np
import matplotlib.pyplot as plt
import scipy


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


def plot_GT_pred(img, bbox, bbox_gt, mask = [], color='c', color_gt='g'):
  plt.imshow(img)

  img_height = np.size(img, 0)
  img_width = np.size(img, 1)

  for i_ in range(len(bbox)):
    bb_i = bbox[i_]
    #lbl_i = lbl[i_]
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
    #str_test = 'p = ' + str(round(lbl_i[2], 3))
    #plt.text(x2, y2, str_test, color=color)
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

  if len(mask) > 1:
    ymin = int(np.array(bbox[0])[[1,3,5,7]].min())
    xmin = int(np.array(bbox[0])[[0,2,4,6]].min())
    ymax = int(np.array(bbox[0])[[1,3,5,7]].max())
    xmax = int(np.array(bbox[0])[[0,2,4,6]].max())
    w = int(xmax - xmin)
    h = int(ymax - ymin)

    mask = scipy.misc.imresize(mask, (h, w), interp='bilinear')
    new_mask = np.zeros((img_height, img_width))
    new_mask[ymin:(ymin + h), xmin:(xmin + w)] = mask

    plt.imshow(new_mask, alpha=0.5)

  plt.axis('off')
