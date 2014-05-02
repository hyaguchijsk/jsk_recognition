from matplotlib import pyplot
import rospkg

def randstr(n):
  import string
  import random
  alphabets = string.digits + string.letters
  return ''.join(random.choice(alphabets) for i in xrange(n))

def show_image(imgs, window_name=None, set_on_mouse=True, draw_grid=False):
  if imgs.__class__ != 'tuple':
    imgs = (imgs,)
  num = 0
  for img in imgs:
    if window_name == None:
      window_name = randstr(8)
    img_display = img.copy()

    window_name += "-%d"%(num)

    # draw grid
    if draw_grid:
      for i in range(img.shape[0]):
        for j in range(img.shape[1]):
          if (j%100==0 or i %100==0):
            img_display[i, j] = (255, 255, 255) if len(img_display.shape)==3 else 255

    cv2.imshow(window_name, img_display)
    if set_on_mouse:
      cv2.setMouseCallback(window_name, on_mouse, img_display)
    num += 1

def update_scene():
  global camera
  global color, depth, depth_world, normal, mask
  for val in ["color", "depth", "depth_world", "normal", "mask"]:
    try:
      exec(val+"=camera."+val, globals())
    except:
      rospy.loginfo("update_scene: error in "+val)

def depth2gray(depth, min_mm=300, max_mm=1000, margin=10, focus=True):
  # depth max is 4095
  try:
    gray = depth.copy()
    if focus:
      if min_mm < gray[gray.nonzero()].min():
        min_mm = gray[gray.nonzero()].min()
      if gray.max() < max_mm:
        max_mm = gray.max()
    gray[(gray<min_mm) | (max_mm<gray)] = 0
    gray[gray.nonzero()] = (gray[gray.nonzero()]-min_mm)/(max_mm-min_mm)*(255-margin) + margin
    return np.uint8(gray)
  except:
    return np.zeros(depth.shape, dtype=np.uint8)

def diff2gray(diff, min_mm=2, max_mm=10, max_cut_mm=100, margin=0):
  # depth max is 4095
  gray = diff.copy()
  gray[gray>max_cut_mm] = 0
  gray[gray>max_mm] = max_mm
  gray[gray<min_mm] = 0
  # gray[gray.nonzero()] += PLANE_HEIGHT * 1000.0
  gray[gray.nonzero()] = (gray[gray.nonzero()]-min_mm)/(max_mm-min_mm)*(255-margin)
  # filter
  # gray = cv2.bilateralFilter(np.uint8(gray.copy()), 0, 32, 2)
  # gray = cv2.GaussianBlur(gray.copy(), (9,9), 2, 2)
  return np.uint8(gray)

def remove_outer(img, udlr=(30, 100, 50, 50)):
  if udlr.__class__ == int:
    udlr = (udlr, ) * 4
  res = img.copy()
  u, d, l, r = udlr
  height, width = img.shape[:2]

  res[:u,:] = 0
  res[height-d:,:] = 0
  res[:,:l] = 0
  res[:,width-r:] = 0
  return res

def on_mouse(event, x, y, flags, img):
  if not (0<=x<img.shape[1] and 0<=y<img.shape[0]): return
  global camera

  if event==cv2.EVENT_LBUTTONDOWN:
  # if flags == cv2.EVENT_FLAG_LBUTTON:
    print "*** pixel information ***"
    print "clicked x:%s, y:%s"%(x, y)
    print "color_bgr is", camera.color[y, x]
    print "color_hsv is", camera.color_hsv[y, x]
    print "depth is", camera.depth[y, x]
    print "depth_world is", camera.depth_world[y, x]
    print "xyz_world is", camera.xyz_world[y, x]
    print "img val is", img[y, x]

  elif flags == cv2.EVENT_FLAG_RBUTTON:
    print "*save_img*"
    cv2.imwrite("jpg/"+str(time.time())+".jpg", img)

def save_img(img):
  cv2.imwrite("jpg/"+str(time.time())+".jpg", img)

def save_array(img, name=None):
  if not name:
    name = str(time.time())
  np.save("npy/"+name, img)

def do_mask(img, mask):
  if len(img.shape) == 2:
    return img * mask
  else:
    return img * dstack_mask(mask)

def find_all_zero(img_3col):
  mask =  (img_3col[:,:,0] == 0)
  mask *= (img_3col[:,:,1] == 0)
  mask *= (img_3col[:,:,2] == 0)
  return mask

def publish_tf_theta(xyz, theta, target_frame="/pick_position", camera_frame="/BASE"):
  quat = tf.transformations.quaternion_from_euler(0, 0, theta)
  camera._tf_broadcaster.sendTransform(xyz, quat, rospy.Time.now(), target_frame, camera_frame)

def publish_tf(xyz, rpy=(0,0,0), target_frame="/pick_position", camera_frame="/BASE"):
  quat = tf.transformations.quaternion_from_euler(rpy[0], rpy[1], rpy[2])
  camera._tf_broadcaster.sendTransform(xyz, quat, rospy.Time.now(), target_frame, camera_frame)

def get_plane_model():
  pkgpath = rospkg.get_path('clothbag_rim_detector')
  depth_plane_model = np.load(pkgpath + "model/depth_plane_model.npy")
  depth_plane_model[depth_plane_model==0] = PLANE_HEIGHT*1000
  return camera.get_roi(depth_plane_model)


def get_bag_state():
  global mouse_point_left, mouse_point_right, mouse_xyz_left, mouse_xyz_right
  global rect_mask, rotated_rect
  rect_mask = (bag_area_mask * 255).astype("uint8")
  if not rect_mask.any():
    return
  rotated_rect = get_rotated_rectangle(rect_mask)
  cv2.fillConvexPoly(rect_mask, rotated_rect, 255)

  mouse_point_left, mouse_point_right = get_cut_line(bag_area_mask, rotated_rect)
  mouse_xyz_left, mouse_xyz_right = camera.xyz_world[mouse_point_left[1], mouse_point_left[0]], camera.xyz_world[mouse_point_right[1], mouse_point_right[0]]

  global BAG_SIZE, BAG_DIRECTION_POINT, BAG_DIRECTION_XYZ, BAG_DIRECTION_THETA
  try:
    d1 = np.sqrt( ((camera.point2xyz_world(rotated_rect[0]) - camera.point2xyz_world(rotated_rect[1]))**2).sum() )
    d2 = np.sqrt( ((camera.point2xyz_world(rotated_rect[1]) - camera.point2xyz_world(rotated_rect[2]))**2).sum() )
    BAG_SIZE = (d1 + d2)/2
  except:
    pass

  BAG_DIRECTION_POINT = mouse_point_right - mouse_point_left
  BAG_DIRECTION_XYZ = mouse_xyz_right - mouse_xyz_left
  BAG_DIRECTION_THETA = np.arctan(BAG_DIRECTION_XYZ[1]/BAG_DIRECTION_XYZ[0])

def calc_center_xyz(xyz1, xyz2):
  return (np.array(xyz1)+np.array(xyz2))/2

def set_hand(hand):
  rospy.set_param("hand", hand)


def set_hand_correction(dx=0.0, dy=0.0, dz=0.0):
  global hand_xyz_correction
  hand_xyz_correction = np.array((dx, dy, dz), dtype="float")

def slide_hand_xyz(slide_mm=10):
  global mouse_xyz_left, mouse_xyz_right
  global hand_xyz_correction
  try:
    hand_xyz_correction
  except:
    hand_xyz_correction = np.array((0, 0, 0), dtype="float")

  v = mouse_xyz_right - mouse_xyz_left
  v_normal = np.array((1, -v[0]/v[1], 0), dtype="float32")
  v_normal /= np.sqrt((v_normal**2).sum())
  v_slide = slide_mm/1000.0 * v_normal
  hand_xyz_correction += v_slide

def slide_mouse_line(slide_mm=10):
  global mouse_xyz_left, mouse_xyz_right
  global mouse_point_left, mouse_point_right

  v = mouse_xyz_right - mouse_xyz_left
  v_normal = np.array((1, -v[0]/v[1], 0), dtype="float32")
  v_normal /= np.sqrt((v_normal**2).sum())
  v_slide = slide_mm/1000.0 * v_normal

  mouse_xyz_left += v_slide
  mouse_xyz_right += v_slide

  mouse_point_left = camera.xyz_world2point(mouse_xyz_left)
  mouse_point_right = camera.xyz_world2point(mouse_xyz_right)
