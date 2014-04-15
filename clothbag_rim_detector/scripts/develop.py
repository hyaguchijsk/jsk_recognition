def get_grabcut_mask(color, on_plane_mask):
  img = color.copy()
  mask = on_plane_mask.copy()
  mask[mask.nonzero()] = 1
  rect = None
  bgdmodel = np.zeros((1,65),np.float64)
  fgdmodel = np.zeros((1,65),np.float64)

  # draw rotated rectangle in mask
  rotated_rect = get_rotated_rectangle(mask)
  try:
    if rotated_rect.any():
      cv2.fillConvexPoly(mask, rotated_rect, 3)
    else:
      pass
  except Exception as e:
    rospy.logerr("GrabCut: can't find rotated rectangle")

  # cv2.ellipse(mask, rotated_rect, 1, -1)
  center_point = get_center_of_mask(mask)
  if center_point != None:
    CIRCLE_SIZE = 10
    cv2.circle(mask, tuple(center_point), CIRCLE_SIZE, 3, -1)

  if mask.any():
    cv2.grabCut(img,mask,rect,bgdmodel,fgdmodel,1,cv2.GC_INIT_WITH_MASK)
    # cv2.grabCut(middle_output,mask,rect,bgdmodel,fgdmodel,1,cv2.GC_EVAL)

    mask = np.where((mask==1) + (mask==3),255,0).astype('uint8')
    # mask = np.where((mask==3),255,0).astype('uint8')
    mask = erode_and_dilate_filter(mask, 3)

  return mask!=0

def mask2points(mask):
  nonzero = mask.nonzero()
  return np.dstack((nonzero[1], nonzero[0]))

def get_center_of_mask(mask):
  xy = camera.xyz_world[:,:,:2][mask.nonzero()]
  if xy.any():
    mean_xyz, eigenvector = cv2.PCACompute(xy)
    global mean_point
    mean_point = np.reshape( camera.xyz_world2point(mean_xyz[0,0], mean_xyz[0,1], PLANE_HEIGHT), (1,2) )
    return mean_point[0]
  else:
    return None

def get_rotated_rectangle(mask):
  if mask.any():
    rect = cv2.minAreaRect(mask2points(mask))
    box = cv2.cv.BoxPoints(rect)
    box = np.int0(box)
    return box
  else:
    return np.array(())


def get_rect_size(mask, co=None):
  box = get_rotated_rectangle(mask)
  if box.any():
    p1, p2, p3, p4 = box
  else:
    rospy.logerr("rotated rectangle not found. return 50")
    return 50
  tmp1 = (p1 - p2)**2
  tmp2 = (p2 - p3)**2
  l1 = np.sqrt(tmp1[0] + tmp1[1])
  l2 = np.sqrt(tmp2[0] + tmp2[1])

  # print int(max(l1, l2)), max(l1, l2)/min(l1, l2)
  # (MAX_SIZE, rate)
  # (461, 1.46)->2.4, (369, 1.01)->2.1, (320, 1.11)->1.7
  # (250, 1.01)->1.6, (300, 1.5)->1.8, (270. 1.17)->1.7
  MAX_SIZE, RATE = int(max(l1, l2)), max(l1, l2)/min(l1, l2)
  if not co:
    co = 1.6 + (MAX_SIZE-250) * 0.005
  return int(MAX_SIZE/co)

def get_cut_line(bag_area_mask, rotated_rect):
  bottom, left_of_bottom = rotated_rect[:2]
  v = bottom - left_of_bottom
  tilt = v[1]*1.0/v[0]
  print "tilt:", tilt, ", th:", 0.3
  if tilt > 0.3:
    # rotated left
    lb, lt, rt, rb = rotated_rect
  else:
    # rotated right
    rb, lb, lt, rt = rotated_rect

  line_vector = rb - lb
  line_vector_normal = lt - lb
  length = np.sqrt( (line_vector_normal**2).sum() )

  i_min = 100
  i_max = 0
  for i in range(100):
    cut_rate = i/100.0
    inner_rate1 = 0.1
    inner_rate2 = 0.3
    cut_point_left = (lb + line_vector_normal*cut_rate).astype("int")
    inner_point_left1 = (cut_point_left + inner_rate1*line_vector).astype("int")
    inner_point_left2 = (cut_point_left + inner_rate2*line_vector).astype("int")
    cut_point_right = (rb + line_vector_normal*cut_rate).astype("int")
    inner_point_right1 = (cut_point_right - inner_rate1*line_vector).astype("int")
    inner_point_right2 = (cut_point_right - inner_rate2*line_vector).astype("int")

    # if i == 34:
    # if False: # debug
    #   hoge = np.zeros((480,640,3), dtype="uint8")
    #   hoge[:,:,0] = (bag_area_mask*255).astype("uint8")
    #   hoge[:,:,1] = (bag_area_mask*255).astype("uint8")
    #   hoge[:,:,2] = (bag_area_mask*255).astype("uint8")
    #   hoge = draw_points(hoge, [cut_point_left, cut_point_right], (255, 0, 0))
    #   hoge = draw_points(hoge, [inner_point_left1, inner_point_right1], (0, 255, 0))
    #   hoge = draw_points(hoge, [inner_point_left2, inner_point_right2], (0, 0, 255))
    #   save_img(hoge)
    #   show_image(hoge, "hoge", False, False)

    try:
      if bag_area_mask[inner_point_left1[1], inner_point_left1[0]] and bag_area_mask[inner_point_right1[1], inner_point_right1[0]]:
        if bag_area_mask[inner_point_left2[1], inner_point_left2[0]] and bag_area_mask[inner_point_right2[1], inner_point_right2[0]]:
          print i
          return cut_point_left, cut_point_right
    except:
      pass


  return lb, rb


