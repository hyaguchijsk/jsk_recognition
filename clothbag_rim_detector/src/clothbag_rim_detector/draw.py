def draw_line(img, line_param=None, line_color=(255,0,0)):
  result_color = img.copy()

  if line_param:
    # xyz
    a, b, c = calc_abc_from_line_param(line_param)
    # pixel
    line_func_x = (lambda x: int(-a/b*x + -c/b)) if b != 0 else None
    line_func_y = (lambda y: int(-b/a*y + -c/a)) if a != 0 else None

    # draw to end_point
    if line_func_x:
      range_x = range(img.shape[1])
      for x in range_x:
        cv2.circle(result_color, (x, line_func_x(x)), 1, line_color, thickness=-1)

    if line_func_y:
      range_y = range(img.shape[0])
      for y in range_y:
        cv2.circle(result_color, (line_func_y(y), y), 1, line_color, thickness=-1)
  return result_color

def draw_contour(img, contour, contour_color=(0,0,255)):
  result_color = img.copy()
  cv2.drawContours(result_color, contour, -1, contour_color, thickness=3)
  return result_color

def get_max_contour(contour, max_len=50):
  for c in contour:
    if len(c) > max_len:
      max_len = len(c)
  res = []
  for c in contour:
    if len(c) == max_len:
      res.append(c)
  return res

def draw_points(img, points, point_color=(0,0,255)):
  result_color = img.copy()
  if len(np.array(points).shape) == 1:
    points = np.reshape(points, (1,2))
  for point in points:
    cv2.circle(result_color, tuple(point), 5, point_color, thickness=-1)
  return result_color

def plot_points(points):
  pyplot.plot(points[:,0], points[:,1])
  pyplot.show()

def draw_rotated_rectangle(color, box, point_color=(0,0,255)):
  res = color.copy()
  cv2.drawContours(res,[box],0,point_color,2)
  return res

def calc_abc_from_line_param(line_param):
  if line_param is None:
    return None
  line_param = np.array(line_param)
  # line: 1/vx(x-cx)=1/vy(y-cy)
  if len(line_param) == 4:
    vx, vy, cx, cy = line_param
    # x/vx - y/vy + cy/vy - cx/vx = 0
    a, b, c = 1/vx, -1/vy, cy/vy - cx/vx
  elif len(line_param) == 3:
    a, b, c = line_param
  elif len(line_param) == 2:
    a, b, c = line_param[0], -1, line_param[1]
  return np.array((a, b, c))

def draw_text(color_edit):
  global BAG_DIRECTION_POINT, BAG_DIRECTION_XYZ, BAG_SIZE, GRASP_STATE, TASK_STAGE
  res = color_edit.copy()
  text_area = np.zeros(camera.color.shape[:2], dtype="bool")
  size_xy = (250, 100)
  start_y, start_x = (480-size_xy[1], 0)
  # start_y, start_x = (0, 0)
  text_area[start_y:start_y+size_xy[1], start_x:start_x+size_xy[0]] = True
  text_area[start_y:start_y+size_xy[1]-40, start_x+160:start_x+size_xy[0]] = False 
  res[text_area] = (0,0,0)

  TASK_STAGE = rospy.get_param("task_stage", "None")

  cv2.putText(res, "GRASP_STATE:", (start_x+10,start_y+20), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 2)
  cv2.putText(res, str(GRASP_STATE), (start_x+10,start_y+50), cv2.FONT_HERSHEY_PLAIN, 2, (0,255,255), 2)
  cv2.putText(res, "TASK_STAGE:", (start_x+10,start_y+70), cv2.FONT_HERSHEY_PLAIN, 1, (0,0,255), 2)
  cv2.putText(res, str(TASK_STAGE), (start_x+10,start_y+95), cv2.FONT_HERSHEY_PLAIN, 2, (255,255,0), 2)
  # cv2.putText(res, str(BAG_DIRECTION_POINT), (10,60), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2)
  # cv2.putText(res, str(int(BAG_SIZE*1000))+"mm", (10,80), cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2)

  return res
