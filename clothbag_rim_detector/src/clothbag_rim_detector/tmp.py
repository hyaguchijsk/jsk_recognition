def get_hand_xyz():
  global mouse_xyz_left, mouse_xyz_right
  try:
    v = mouse_xyz_right - mouse_xyz_left
    v_normal = np.array((1, -v[0]/v[1], 0), dtype="float32")
    v_normal /= np.sqrt((v_normal**2).sum())

    middle = mouse_xyz_left + 0.5*v
    d = 0.10
    z = 0.10
    middle += d*v_normal
    middle[2] += z
    return middle
  except Exception as e:
    rospy.logerr("get_hand_xyz: error")
    rospy.logerr(e)
    return (0,0,0)

# calc from depth_max
def get_hand_point(color):
  img = color.copy()
  mask = -find_all_zero(bag_area_color)
  z_max = camera.depth_world.max()
  mask[camera.depth_world<z_max-20] = 0
  mask[camera.depth_world>z_max-10] = 0
  mask = get_erode_and_dilate_mask(img*dstack_mask(mask), 0)[:,:,0]
  x = mask.nonzero()[1]
  y = mask.nonzero()[0]
  try:
    return int(x.mean()), int(y.mean())
  except Exception as e:
    rospy.logerr("get_hand_point: error")
    rospy.logerr(e)
    return (0,0)


def get_contour(color):
  gray_img = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
  contours, hierarchy = cv2.findContours(gray_img.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
  return contours

def check_point():
  global POINT1, POINT2, POINT3
  print "POINT1: ", POINT1, camera.xyz_world[POINT1[1], POINT1[0]]
  # print "POINT2: ", POINT2, camera.xyz_world[POINT2[1], POINT2[0]]
  # print "POINT3: ", POINT3, camera.xyz_world[POINT3[1], POINT3[0]]

# def tmp_corner_and_direction():
    # Todo: pixel to 3d
    # corners = get_corners_of_bag(bag_area_color)
    # if corners.size:
    #   color_edit = draw_points(color_edit, corners)

    # bag_direction
    # param1, param2 = get_bag_direction(bag_area_color)
    # color_edit = draw_line(color_edit, line_param=param1, line_color=(0,0,255))
    # color_edit = draw_line(color_edit, line_param=param2, line_color=(0,255,0))

# def get_bag_direction(bag_area_color):
#   mask_image = bag_area_color[:,:,0].nonzero()
# 
#   d = np.dstack((mask_image[1], mask_image[0]))[0].astype(np.float32)
#   if d.size==0:
#     return None, None
#   mean, eigenvector = cv2.PCACompute(d)
#   # x, y
#   c1 = eigenvector[0,0]*mean[0,0] + eigenvector[0,1]*mean[0,1]
#   c2 = eigenvector[1,0]*mean[0,0] + eigenvector[1,1]*mean[0,1]
# 
#   param1 = [eigenvector[1,0], eigenvector[1,1], -c2]
#   param2 = [eigenvector[0,0], eigenvector[0,1], -c1]
# 
#   return param1, param2
# 
# def get_bag_direction2(bag_area_color):
#   mask_image = bag_area_color[:,:,0]
#   if not mask_image.any():
#     return None, None
#   xy = camera.xyz_world[:,:,:2][mask_image.nonzero()]
# 
#   global eigenvector
#   mean_xyz, eigenvector = cv2.PCACompute(xy)
# 
#   mean_pixel = np.reshape( camera.xyz_world2point(mean_xyz[0,0], mean_xyz[0,1], PLANE_HEIGHT), (1,2) )
#   # x, y
#   c1 = eigenvector[0,0]*mean_pixel[0,0] + eigenvector[0,1]*mean_pixel[0,1]
#   c2 = eigenvector[1,0]*mean_pixel[0,0] + eigenvector[1,1]*mean_pixel[0,1]
# 
#   param1 = [eigenvector[1,0], eigenvector[1,1], -c2]
#   param2 = [eigenvector[0,0], eigenvector[0,1], -c1]
# 
#   return param1, param2
# 
# 
# def get_center_of_bag(bag_area_color):
#   mask_image = bag_area_color[:,:,0]
# 
#   xy = camera.xyz_world[:,:,:2][mask_image.nonzero()]
#   mean_xyz, eigenvector = cv2.PCACompute(xy)
#   mean_pixel = np.reshape( camera.xyz_world2point(mean_xyz[0,0], mean_xyz[0,1], PLANE_HEIGHT), (1,2) )
#   return mean_pixel[0]
# 
# def get_corners_of_bag(bag_area_color):
#   mask_image = bag_area_color[:,:,0]
# 
#   contour = get_contour(bag_area_color)
#   # x, y
#   if len(contour) == 0:
#     return np.array([])
# 
#   points = get_max_contour(contour)[0][:,0]
#   center_point_of_bag = get_center_of_bag(bag_area_color)
# 
#   # array of float value
#   direction_array = np.array([], dtype=np.float32)
#   distance_array = np.array([], dtype=np.float32)
#   for point in points:
#     calc_distance = lambda center_point, point: np.sqrt( (center_point[0]-point[0])**2+(center_point[1]-point[1])**2 )
#     calc_direction = lambda center_point, point: np.arctan( (point[1]-center_point[1])/(point[0]-center_point[0]) )
#     direction = calc_direction(center_point_of_bag, point)
#     distance = calc_distance(center_point_of_bag, point)
#     direction_array = np.append(direction_array, direction)
#     distance_array = np.append(distance_array, distance)
# 
#   # array of point (x, y)
#   peak_args = get_peak_arg_of_distance(distance_array)
#   corners = points[peak_args] if peak_args.size>0 else np.array([])
#   return corners
# 
# def get_peak_arg_of_distance(distance_array):
#   nearest_arg = distance_array.argmin()
#   sorted_array = np.hstack((distance_array[nearest_arg:], distance_array[:nearest_arg]))
#   peaks = np.array([], dtype=np.float32)
# 
#   average_num = 5
#   average_array = np.array([], dtype=np.float32)
#   for i in range(len(sorted_array)):
#     average_array = np.append(average_array, sorted_array[i:i+average_num].mean())
# 
#   peak_margin = len(distance_array)/10
#   last_point = -peak_margin
#   r = 20
#   peak_arg = []
#   for i in range(len(sorted_array) - r):
#     if average_array[i-r] < average_array[i] > average_array[i+r] and average_array[i-1] < average_array[i] > average_array[i+1]:
#       if i < last_point + peak_margin:
#         pass
#       else:
#         peak_arg.append( (i+nearest_arg)%len(distance_array) )
#         last_point = i
#   return np.array(peak_arg)

# around base_point
# def get_nearest_point(value_list, value, base_point):
#   margin = 10
#   if base_point-margin > 0:
#     value_list[:base_point-margin] = 0
#   if base_point+margin < len(value_list):
#     value_list[base_point+margin:] = 0
# 
#   diff_list = abs(value_list-value)
# 
#   return diff_list.argmin()


def rhand():
  rospy.set_param("hand", "rhand")
def lhand():
  rospy.set_param("hand", "lhand")

def pick_bag():
  rospy.set_param("task_stage", "pick_bag")
def change_hand():
  rospy.set_param("task_stage", "change_hand")
def raise_hand():
  rospy.set_param("task_stage", "raise_hand")
