def calc_target_point_xyz(task_stage):
  global mouse_xyz_left, mouse_xyz_right, hand_xyz_world
  global upper_rim, lower_rim, border_rim
  global PICK_BAG_TARGET
  global CHANGE_HAND_TARGET
  global RAISE_HAND_TARGET
  global PUT_OBJECT_TARGET

  v = mouse_xyz_right - mouse_xyz_left
  v /= np.sqrt((v ** 2).sum())

  v2 = hand_xyz_world - mouse_xyz_left
  v2 /= np.sqrt((v2 ** 2).sum())

  v_normal = np.array((1, -v[0] / v[1], 0), dtype = "float32")
  v_normal /= np.sqrt((v_normal ** 2).sum())

  if task_stage == "pick_bag":
    p = mouse_xyz_right - PICK_BAG_TARGET / 1000.0 * v
    camera_xyz_world = camera.xyz_world[lower_rim]
    camera_xyz_world = camera_xyz_world[~np.isnan(camera_xyz_world).any(1)]
    dist_list = calc_dist_point_to_point(camera_xyz_world, p)

    if len(dist_list) == 0:
      rospy.logerr("target_point/pick_bag: lower_rim not found.")
      return

    argmin = (dist_list ** 2).argmin()
    # target_xyz = camera.xyz_world[lower_rim][argmin]
    target_xyz = camera_xyz_world[argmin]

    pre_target_xyz = target_xyz.copy()
    pre_target_xyz[2] += 0.05

    after_target_xyz = target_xyz.copy()
    after_target_xyz[2] += 0.05
    after_target_xyz += 30 / 1000.0 * v_normal

    # rospy.loginfo("  target_xyz: " + str(target_xyz))
    # rospy.loginfo("  pre_target_xyz: " + str(pre_target_xyz))
    # rospy.loginfo("  after_target_xyz: " + str(after_target_xyz))

    publish_tf_theta(target_xyz,
                     BAG_DIRECTION_THETA,
                     "pick_bag_position")
    publish_tf_theta(pre_target_xyz,
                     BAG_DIRECTION_THETA,
                     "pre_pick_bag_position")
    publish_tf_theta(after_target_xyz,
                     BAG_DIRECTION_THETA,
                     "after_pick_bag_position")

    return camera.xyz_world2point(target_xyz)

  elif task_stage == "change_hand":
    p = hand_xyz_world - CHANGE_HAND_TARGET / 1000.0 * v2
    camera_xyz_world = camera.xyz_world[upper_rim]
    camera_xyz_world = camera_xyz_world[~np.isnan(camera_xyz_world).any(1)]
    dist_list = calc_dist_point_to_point(camera_xyz_world, p)

    if len(dist_list) == 0:
      rospy.logerr("target_point/change_hand: upper_rim not found.")
      return

    argmin = (dist_list ** 2).argmin()
    # target_xyz = camera.xyz_world[upper_rim][argmin]
    target_xyz = camera_xyz_world[argmin]
    pre_target_xyz = target_xyz.copy()

    target_xyz[2] -= 0.02
    target_xyz += 10 / 1000.0 * v
    target_xyz -= 10 / 1000.0 * v_normal

    pre_target_xyz[2] += 0.03
    pre_target_xyz -= 20 / 1000.0 * v
    pre_target_xyz -= 20 / 1000.0 * v_normal

    publish_tf_theta(target_xyz,
                     BAG_DIRECTION_THETA,
                     "change_hand_position")
    publish_tf_theta(pre_target_xyz,
                     BAG_DIRECTION_THETA,
                     "pre_change_hand_position")

    return camera.xyz_world2point(target_xyz)

  elif task_stage == "raise_hand":
    target_xyz = hand_xyz_world.copy() - hand_xyz_correction
    reverse_target_xyz = target_xyz.copy()

    target_xyz[2] += 0.01
    target_xyz += RAISE_HAND_TARGET / 1000.0 * v_normal

    reverse_target_xyz[2] -= 0.01
    reverse_target_xyz -= RAISE_HAND_TARGET / 1000.0 * v_normal

    # print target_xyz, hand_xyz_world, np.sqrt(
    #   ((target_xyz-hand_xyz_world)**2).sum())

    publish_tf_theta(target_xyz,
                     BAG_DIRECTION_THETA,
                     "raise_hand_position")
    publish_tf_theta(reverse_target_xyz,
                     BAG_DIRECTION_THETA,
                     "raise_hand_reverse_position")

    return camera.xyz_world2point(target_xyz)


  elif task_stage == "put_object":
    target_xyz = hand_xyz_world
    target_xyz[2] -= (target_xyz[2] - PLANE_HEIGHT) * 4 / 5
    pre_target_xyz = target_xyz.copy()

    target_xyz += PUT_OBJECT_TARGET / 1000.0 * v_normal
    # pre_target_xyz -= 80 / 1000.0 * v_normal
    pre_target_xyz -= 50 / 1000.0 * v_normal  # small_gray

    publish_tf_theta(pre_target_xyz,
                     BAG_DIRECTION_THETA + np.pi/2,
                     "pre_put_object_position")
    publish_tf_theta(target_xyz,
                     BAG_DIRECTION_THETA + np.pi/2,
                     "put_object_position")

    return camera.xyz_world2point(target_xyz)

