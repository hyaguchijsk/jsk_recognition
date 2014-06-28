GET_BAG_STATE = False
def process():
  global camera, depth_plane_model, bag_area_color, bag_area_mask
  global color_edit, depth_world_edit, mask_before
  global GET_BAG_STATE
  global mouse_point_left, mouse_point_right, hand_point
  global mouse_xyz_left, mouse_xyz_right, hand_xyz_world
  global hand, hand_xyz_correction

  SET_ON_MOUSE = False
  DRAW_GRID = False

  try:
    ## initialize ##
    # rospy.loginfo("process: initilize...")
    depth_plane_model = get_plane_model()
    color_edit = camera.color.copy()
    depth_world_edit = camera.depth_world.copy()


    ## mask_before ##
    # rospy.loginfo("process: mask_before...")
    mask_before = diff2gray(camera.depth_world-depth_plane_model, 1, 10, 200, 0)
    mask_before = erode_and_dilate_filter(mask_before, 7)
    # mask_before = remove_outer(mask_before, (0, 20, 0, 350))
    # mask_before = remove_outer(mask_before, (0, 50, 50, 100))
    mask_before = remove_outer(mask_before, (0, 20, 20, 100))
    # show_image(mask_before, "mask_before", SET_ON_MOUSE, DRAW_GRID)

    bag_area_mask = get_grabcut_mask(camera.color, mask_before)
    bag_area_mask = (
      erode_and_dilate_filter(bag_area_mask.astype("uint8"), 7)).astype("bool")
    bag_area_color = camera.color * dstack_mask(bag_area_mask)


    ## box ##
    # rospy.loginfo("process: box...")
    box = get_rotated_rectangle(bag_area_mask)
    if box.any():
      color_edit = draw_rotated_rectangle(color_edit, box, (0, 0, 255))

    if GET_BAG_STATE:
      get_bag_state()
    try:
      mouse_xyz_left, mouse_xyz_right
    except:
      get_bag_state()


    # rospy.loginfo("process: hand position...")
    hand = rospy.get_param("hand", "None_right")
    if hand=="rhand":
      hand_xyz_world = camera.rhand_xyz_world
      hand_xyz_correction = np.array([0.0, 0.025, 0])
      # hand_xyz_correction = np.array([0.0, 0.0, 0])
    elif hand=="lhand":
      hand_xyz_world = camera.lhand_xyz_world
      hand_xyz_correction = [-0.02, -0.01, 0.06]
    else:
      hand_xyz_world = get_hand_xyz()
    # hand pos correction
    try:
      hand_xyz_world += hand_xyz_correction
    except:
      pass

    # rospy.loginfo("hand_xyz_world: ")
    # rospy.loginfo(str(hand_xyz_world))
    hand_point = camera.xyz_world2point(hand_xyz_world)


    # rospy.loginfo("process: get rims...")
    global upper_rim, lower_rim, border_rim
    upper_rim, lower_rim, border_rim = get_rims()

    color_edit[border_rim] = (0, 0, 255)
    color_edit[upper_rim] = (255, 255, 0)
    color_edit[lower_rim] = (0, 255, 255)

    color_edit = draw_points(color_edit, hand_point, (0, 255, 0))

    # plane_distance
    # rospy.loginfo("process: plane distance color...")
    global plane_distance_color
    plane_distance = calc_dist_point_to_plane(camera.xyz_world,
                                              plane_param,
                                              True)
    plane_distance_color = np.zeros(camera.color.shape, dtype = "uint8")
    d_margin = 0.03
    plane_distance_color[plane_distance < -d_margin] = (255, 0, 0)
    plane_distance_color[abs(plane_distance) < d_margin] = (0, 255, 0)
    plane_distance_color[plane_distance > d_margin] = (0, 0, 255)
    plane_distance_color *= dstack_mask(bag_area_mask)

    # rospy.loginfo("process: in plane color...")
    in_plane_color = np.zeros(camera.color.shape, dtype = "uint8")
    in_plane_color[abs(plane_distance) < d_margin] = (0, 255, 0)
    in_plane_color[plane_distance > d_margin] = (0, 255, 0)
    in_plane_color *= dstack_mask(bag_area_mask)
    in_plane_mask = -find_all_zero(in_plane_color)
    in_plane_mask = remove_outer(in_plane_mask, (50, 50, 50, 50))

    # rospy.loginfo("process: in plane contour...")
    in_plane_contour = get_contour(in_plane_color)
    # cv2.fillConvexPoly(in_plane_color,
    #                    in_plane_contour[0],
    #                    (0, 255, 0),
    #                    lineType = 4)
    in_plane_color = draw_contour(in_plane_color, in_plane_contour)

    # rospy.loginfo("process: in plane mask...")
    y_argmax = np.zeros(in_plane_mask.shape[1], dtype = "int")
    y_argmin = np.zeros(in_plane_mask.shape[1], dtype = "int")
    for i in range(in_plane_mask.shape[1]):
      tmp = in_plane_mask[:, i].nonzero()[0]
      if len(tmp):
        y_argmax[i] = tmp.max()
        y_argmin[i] = tmp.min()
        in_plane_mask[y_argmin[i]:y_argmax[i], i] = True

    # rospy.loginfo("process: plane distance color...")
    plane_distance_color[
      (plane_distance < -d_margin) * (in_plane_mask)] = (255, 0, 255)
    # plane_distance_color *= dstack_mask(in_plane_mask)

    # show_image(plane_distance_color, "plane_distance_color", False)
    # show_image((in_plane_mask * 255).astype("uint8"), "in_plane_mask", False)
    # show_image(in_plane_color, "in_plane_color", False)

    # rospy.loginfo("process: draw points...")
    try:
      color_edit = draw_points(color_edit, mouse_point_left, (255, 0, 0))
      color_edit = draw_points(color_edit, mouse_point_right, (0, 0, 255))
      # cv2.line(color_edit,
      #          tuple(mouse_point_left),
      #          tuple(mouse_point_right),
      #          (0, 255, 255),
      #          3)
    except Exception as e:
      rospy.logerr(e)
      rospy.logerr("Please find mouse by function get_bag_state")

    # # canny
    # depth = depth_world_edit * bag_area_mask
    # xy = depth.nonzero()
    # (x_min,
    #  y_min,
    #  x_max,
    #  y_max) = (xy[1].min(),
    #            xy[0].min(),
    #            xy[1].max(),
    #            xy[0].max())
    # # depth[depth==0] = PLANE_HEIGHT*1000
    # canny = cv2.Canny(depth2gray(depth),100,255)
    # show_image(canny, "canny", False)

    # hand
    # color_edit = draw_points(
    #   color_edit, [camera.rhand_point, camera.lhand_point], (0,255,0))

    global c
    c = color_edit.copy()

    # TEXT
    # rospy.loginfo("process: get grasp state...")
    global GRASP_STATE
    GRASP_STATE = pick_judge(upper_rim, lower_rim)
    color_edit = draw_text(color_edit)

    # ROS PARAM
    # rospy.loginfo("process: set grasp state...")
    rospy.set_param("grasp_state", GRASP_STATE)
    # rospy.set_param("bag_type", BAG_TYPE)  # which bag

    # pick_point
    rospy.loginfo("process: pick point...")
    task_stage = rospy.get_param("task_stage", "None")
    rospy.loginfo("  task_stage: " + task_stage)
    target_point = calc_target_point_xyz(task_stage)
    if target_point is not None:
      color_edit = draw_points(color_edit, target_point, (255, 0, 255))

    ### end of process ###
    pub_color_edit.publish(bridge.cv2_to_imgmsg(color_edit, encoding = "bgr8"))

    ## SHOW ##
    # show_image(camera.color, "camera.color", SET_ON_MOUSE, DRAW_GRID)
    show_image(color_edit, "color_edit", SET_ON_MOUSE, DRAW_GRID)
    show_image(bag_area_color, "bag_area_color", SET_ON_MOUSE, DRAW_GRID)
    # show_image(depth2gray(depth_world_edit),
    #            "depth_world_edit", SET_ON_MOUSE, DRAW_GRID)
    # show_image(depth2gray(depth_world_edit, 750, 850, focus = False),
    #            "depth_cut", SET_ON_MOUSE, DRAW_GRID)
    # show_image(depth2gray(depth_plane_model, 710, 750, focus = False),
    #            "depth_plane_model", SET_ON_MOUSE, DRAW_GRID)

  except Exception as e:
    rospy.logerr(e)


