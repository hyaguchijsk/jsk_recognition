def get_rims():
  global mouse_xyz_left, mouse_xyz_right, hand_xyz_world, plane_param
  global hand
  # ax + by + cz + d = 0
  plane_param = solve_plane_equation(hand_xyz_world, mouse_xyz_left, mouse_xyz_right)
  # plane_param[3] = -0.3

  if hand == "lhand" or hand == "None_left":
    ur_line, ur_line_pr = extract_rim_line(bag_area_color, camera.xyz_world, hand_xyz_world, mouse_xyz_right, 40/1000.0, fill_lacking=True)
  else:
    ur_line, ur_line_pr = extract_rim_line(bag_area_color, camera.xyz_world, hand_xyz_world, mouse_xyz_left, 40/1000.0, fill_lacking=True)
  ur_plane, ur_plane_pr = extract_rim_plane(bag_area_color, camera.xyz_world, plane_param, 30/1000.0)

  mask_ur = np.zeros(camera.color.shape[:2], dtype="uint8")
  mask_ur[ur_line_pr] += 1
  mask_ur[ur_plane] += 2
  mask_ur[mask_ur==3] += 100
  mask_ur[ur_line] += 100


  lr_line, lr_line_pr = extract_rim_line(bag_area_color, camera.xyz_world, mouse_xyz_right, mouse_xyz_left, 40/1000.0)
  lr_plane , lr_plane_pr = ur_plane, ur_plane_pr
  mask_lr = np.zeros(camera.color.shape[:2], dtype="uint8")
  mask_lr[lr_line_pr] += 1
  mask_lr[lr_plane] += 2
  mask_lr[mask_lr==3] += 100
  mask_lr[lr_line] += 100


  br = (mask_ur>100) * (mask_lr>100)
  ur = (mask_ur>100) * (-br)
  lr = (mask_lr>100) * (-br)
  return ur, lr, br


# ax + by + cz + d = 0
def solve_plane_equation(point1, point2, point3):
    vector12 = point2 - point1
    vector13 = point3 - point1
    # calc normal vector
    normal_vector = scipy.cross(vector12, vector13)
    # normal vector to unit vector
    norm = np.sqrt(scipy.dot(normal_vector, normal_vector))
    unit_normal_vector = normal_vector / norm
    d = -(unit_normal_vector[0]*point1[0] +
           unit_normal_vector[1]*point1[1] +
           unit_normal_vector[2]*point1[2])
    # return a, b, c, d
    return [unit_normal_vector[0], unit_normal_vector[1], unit_normal_vector[2], d]

def calc_dist_point_to_point(point_mat, point):
  diff_mat = point_mat - point
  if len(point_mat.shape) == 3:
    func_len = lambda mat: np.sqrt((mat[:,:,0]**2)+(mat[:,:,1]**2)+(mat[:,:,2]**2))
  if len(point_mat.shape) == 2:
    func_len = lambda mat: np.sqrt((mat[:,0]**2)+(mat[:,1]**2)+(mat[:,2]**2))
  return func_len(diff_mat)

def calc_dist_point_to_3dline(point_mat, p_line_start, p_line_end):
  line_vector = p_line_end - p_line_start
  point_vector = point_mat - p_line_start
  # mat_cross = np.cross(line_vector[:,:,np.newaxis], point_vector[np.newaxis,:])
  mat_cross = np.cross(line_vector, point_vector)
  func_len = lambda mat: np.sqrt((mat[:,:,0]**2)+(mat[:,:,1]**2)+(mat[:,:,2]**2))
  return func_len(mat_cross)/func_len(line_vector)

def calc_dist_point_to_plane(point_mat, plane_param, direction=False):
  # |ax+by+cz+d|/sqrt(a**2+b**2+c**2)
  a, b, c, d = plane_param
  x, y, z = point_mat[:,:,0], point_mat[:,:,1], point_mat[:,:,2],
  if direction:
    return (a*x + b*y + c*z + d)/np.sqrt(a**2 + b**2 + c**2)
  else:
    return abs(a*x + b*y + c*z + d)/np.sqrt(a**2 + b**2 + c**2)

# extract_rim
def extract_rim_line(color_seg, xyz_world, point_base_xyz, point_corner_xyz, r=30/1000.0, fill_lacking=False):
  func = lambda xyz_array: calc_dist_point_to_3dline(xyz_array, point_base_xyz[np.newaxis, np.newaxis, :], point_corner_xyz[np.newaxis, np.newaxis, :])
  point_base = camera.xyz_world2point(point_base_xyz[0], point_base_xyz[1], point_base_xyz[2])
  point_corner = camera.xyz_world2point(point_corner_xyz[0], point_corner_xyz[1], point_corner_xyz[2])

  margin = 10
  lt_x = min(point_base[0], point_corner[0]) - margin
  rb_x = max(point_base[0], point_corner[0]) + margin
  lt_y = min(point_base[1], point_corner[1]) - margin
  rb_y = max(point_base[1], point_corner[1]) + margin

  color_near_points = color_seg.copy()
  color_near_points[:lt_y, :] = 0
  color_near_points[rb_y:, :] = 0
  color_near_points[:, :lt_x] = 0
  color_near_points[:, rb_x:] = 0

  xyz_world_edit = xyz_world.copy()
  # fill lacking
  if fill_lacking:
    xyz_world_edit[:lt_y, :] = -1
    xyz_world_edit[rb_y:, :] = -1
    xyz_world_edit[:, :lt_x] = -1
    xyz_world_edit[:, rb_x:] = -1
    z_view = xyz_world_edit[:,:,2]
    xyz_world_edit[z_view==0] = point_base_xyz.copy()

  return extract_rim(color_near_points, xyz_world_edit, func, r)

def extract_rim_plane(color_seg, xyz_world, plane_param, r=30/1000.0):
  func = lambda xyz_array: calc_dist_point_to_plane(xyz_array, plane_param)
  return extract_rim(color_seg, xyz_world, func, r)

def extract_rim(color_seg, xyz_world, func, r=30/1000.0):
  seg_area = (-find_all_zero(color_seg)).nonzero()
  if len(seg_area[0])==0:
    return np.zeros(color_seg.shape[:2], dtype="bool"), np.zeros(color_seg.shape[:2], dtype="bool")
  margin = 10
  lt = (max(0,seg_area[0].min()-margin), max(0,seg_area[1].min()-margin) )
  rb = (min(480, seg_area[0].max()+margin), min(640, seg_area[1].max()+margin))
  res_xyz = xyz_world.copy()

  mat_dist = func(res_xyz[lt[0]:rb[0], lt[1]:rb[1]])
  mask_rim = np.zeros(color_seg.shape[:2])
  mask_rim_probably = np.zeros(color_seg.shape[:2])

  mask_rim[lt[0]:rb[0], lt[1]:rb[1]] = (mat_dist < r/3)
  mask_rim_probably[lt[0]:rb[0], lt[1]:rb[1]] = (mat_dist < r)

  mask_rim *= -find_all_zero(color_seg)
  mask_rim_probably *= -find_all_zero(color_seg)

  return (mask_rim!=0), (mask_rim_probably!=0)

def pick_judge(ur, lr):
  global PLANE_HEIGHT, RATE
  ur_size = len(ur.nonzero()[0])
  lr_size = len(lr.nonzero()[0])
  tmp = camera.xyz_world[ur][:,2]
  ur_z_margin = 0.02

  ur_cut = ur * (camera.xyz_world[:,:,2] > (PLANE_HEIGHT + ur_z_margin))
  # ur_cut = ur * (camera.xyz_world[:,:,2] == 0)
  ur_cut_size = len(ur_cut.nonzero()[0])

  th_lr, th_ur, th_ur_cut = RATE * np.array((100, 300, 200))
  # th_lr, th_ur, th_ur_cut = RATE * np.array((500, 400, 300))

  flag1 = lr_size > th_lr
  flag2 = ur_size > th_ur
  flag3 = ur_cut_size > th_ur_cut
  # print (lr_size, ur_size, ur_cut_size), (th_lr, th_ur, th_ur_cut)
  # print flag1, flag2, flag3
  if flag1 and not(flag2 or flag3):
    return "NO"
  if flag1 and flag2 and flag3:
    return "SINGLE"
  if not flag1 and flag2:
    return "BOTH"
  else:
    # rospy.logerr("pick_judge: EXCEPTION")
    # print (lr_size, ur_size, ur_cut_size), (th_lr, th_ur, th_ur_cut)
    # print flag1, flag2, flag3
    return "UNKNOWN"
