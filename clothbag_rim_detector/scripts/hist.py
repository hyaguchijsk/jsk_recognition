def save_hist(color, num, save=True, hist_dir="hist/"):
  hist_bgr = get_hist(color, False)
  hist_bgr_raw = get_hist(color, False, True)
  hist_img_bgr = draw_hist(hist_bgr, False)
  hist_hsv = get_hist(color, True)
  hist_hsv_raw = get_hist(color, True, True)
  hist_img_hsv = draw_hist(hist_hsv, False)

  if save:
    commands.getoutput("mkdir "+hist_dir+str(num))
    dirs = commands.getoutput("ls "+hist_dir+str(num)).split("\n")
    if dirs != ['']:
      dirs = [int(i) for i in dirs]
      dirs.sort()
    max_dir_num = 0 if dirs[-1]=='' else int(dirs[-1])
    new_dir = str(max_dir_num + 1)
    path = hist_dir + str(num) + "/" + new_dir
    commands.getoutput("mkdir " + path)

    cv2.imwrite(path + "/area.jpg", color)
    np.save(path+ "/depth_world", camera.depth_world)
    np.save(path+ "/color", camera.color)
    cv2.imwrite(path+ "/hist_img_bgr.jpg", hist_img_bgr)
    cv2.imwrite(path+ "/hist_img_hsv.jpg", hist_img_hsv)
    np.save(path+ "/hist_bgr.npy", hist_bgr)
    np.save(path+ "/hist_bgr_raw.npy", hist_bgr_raw)
    np.save(path+ "/hist_hsv.npy", hist_hsv)
    np.save(path+ "/hist_hsv_raw.npy", hist_hsv_raw)
  else:
    return hist_bgr, hist_hsv

def load_hist(num, hist_dir):
  dir_path = get_hist_path(num, hist_dir)

  hist_bgr = np.load(dir_path + "/hist_bgr.npy").astype("float32")
  hist_hsv = np.load(dir_path + "/hist_hsv.npy").astype("float32")

  return hist_bgr, hist_hsv

def smooth_hist(hist, margin=5):
  res = hist.copy()
  for ch in (0,1,2):
    for i in range(len(hist)):
      low = max(0, i-margin)
      high  = min(256, i+margin)
      res[i, ch] = hist[:,ch][low:high].mean()
  return res

def get_hist_path(s, hist_dir="hist"):
  num1, num2 = s.split(".")
  dir_path = hist_dir + num1 + "/" + num2
  return dir_path

def draw_saved_hist(num1, num2, smoothing=True, hist_dir="hist/"):
  hist_bgr1, hist_hsv1 = load_hist(num1, hist_dir)
  hist_bgr2, hist_hsv2 = load_hist(num2, hist_dir)

  if smoothing:
    hist_bgr1 = smooth_hist(hist_bgr1)
    hist_bgr2 = smooth_hist(hist_bgr2)
    hist_hsv1 = smooth_hist(hist_hsv1)
    hist_hsv2 = smooth_hist(hist_hsv2)

  draw_hist(hist_bgr1, draw=True, name="hist_bgr1")
  draw_hist(hist_bgr2, draw=True, name="hist_bgr2")
  draw_hist(hist_hsv1, draw=True, name="hist_hsv1")
  draw_hist(hist_hsv2, draw=True, name="hist_hsv2")


def match_hist(in1, in2, method=None, smoothing=True, show=True, hist_dir="hist/"):
  if not method:
    method = cv2.cv.CV_COMP_CORREL #Correlation
    # method = cv2.cv.CV_COMP_CHISQR #Chi-Square
    # method = cv2.cv.CV_COMP_INTERSECT #Intersection
    # method = cv2.cv.CV_COMP_BHATTACHARYYA #Bhattacharyya distance
    # method = cv2.cv.CV_COMP_HELLINGER #Synonym for CV_COMP_BHATTACHARYYA
  if in1.__class__ == str:
    hist_bgr, hist_hsv = load_hist(in1, hist_dir)
    dir1 = get_hist_path(in1, hist_dir)
    img1 = cv2.imread(dir1+"/area.jpg")
  else: # if color image
    hist_bgr = get_hist(in1, False)
    hist_hsv = get_hist(in1, True)
    img1 = in1.copy()
  saved_hist_bgr, saved_hist_hsv = load_hist(in2, hist_dir)
  dir2 = get_hist_path(in2, hist_dir)
  img2 = cv2.imread(dir2+"/area.jpg")

  if smoothing:
    hist_bgr = smooth_hist(hist_bgr)
    saved_hist_bgr = smooth_hist(saved_hist_bgr)
    hist_hsv = smooth_hist(hist_hsv)
    saved_hist_hsv = smooth_hist(saved_hist_hsv)

  compare_bgr = cv2.compareHist(hist_bgr, saved_hist_bgr, method)
  compare_hsv = cv2.compareHist(hist_hsv, saved_hist_hsv, method)

  if show:
    draw_hist(hist_bgr, name="hist1_bgr")
    draw_hist(hist_hsv, name="hist1_hsv")
    draw_hist(saved_hist_bgr, name="hist2_bgr")
    draw_hist(saved_hist_hsv, name="hist2_hsv")
    show_image(img1, "img1")
    show_image(img2, "img2")

  return compare_bgr, compare_hsv


def draw_hist(hist, draw=True, name="hist"):
  if hist.dtype != "uint8":
    hist = hist.astype("uint8")
  bgr = [ (255,0,0),(0,255,0),(0,0,255) ]
  hist_img = np.zeros((256,256,3))
  bins = np.arange(256).reshape(256,1)

  for ch, col in enumerate(bgr):
    pts = np.column_stack((bins,hist[:, ch]))
    cv2.polylines(hist_img,[pts],False,col)
  hist_img = np.flipud(hist_img)
  if draw:
    show_image(hist_img, name, False)
  else:
    return hist_img

def get_hist(color, hsv=True, raw=False):
  img = color.copy() if not hsv else cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
  b = img[:,:,0]
  g = img[:,:,1]
  r = img[:,:,2]
  is_mask = (b!=0) + (g!=0) + (r!=0)
  b = b[is_mask.nonzero()]
  g = g[is_mask.nonzero()]
  r = r[is_mask.nonzero()]
  img = np.dstack((b, g, r))

  # hist = np.zeros((256,3), dtype=np.uint8)
  hist = np.zeros((256,3), dtype=np.float32)

  bins = np.arange(256).reshape(256,1)
  bgr = [ (255,0,0),(0,255,0),(0,0,255) ]
  global hist_item
  for ch, col in enumerate(bgr):
    hist_item = cv2.calcHist([img],[ch],None,[256],[0,256])
    # hist_item[0] = 0
    if not raw:
      cv2.normalize(hist_item, hist_item,0,255,cv2.NORM_MINMAX)
    hist[:, ch] = np.int32(np.around(hist_item))[:, 0]

  return hist

def search_hist(color, threshold=1.6, method=None, smoothing=True, hist_dir="hist/"):
  show_image(color*0, "best_match")
  dirs = commands.getoutput("ls hist").split("\n")
  dirs = [int(i) for i in dirs]
  dirs.sort()
  max_dir_num = 0 if dirs[-1]=='' else int(dirs[-1])

  match_list = []

  max_arg, max_type, max_value = 0, 0, threshold
  for i in range(max_dir_num):
    sub_dirs = commands.getoutput("ls "+hist_dir+str(i+1)).split("\n")
    sub_dirs = [int(k) for k in sub_dirs]
    sub_dirs.sort()
    max_sub_dir_num = 0 if sub_dirs[-1]=='' else int(sub_dirs[-1])
    for j in range(max_sub_dir_num):
      num = str(i+1)+"."+str(j+1)
      compare_result = match_hist(color, num, method, smoothing, False, hist_dir)

      similarity = compare_result[0] + compare_result[1]
      print num + ":", compare_result[0], compare_result[1]
      if similarity > threshold:
        print "high similarity"
        match_list.append(num)
      if similarity > max_value:
        print "update"
        print ""
        max_arg = j+1
        max_type = i+1
        max_value = similarity
  print ""
  which = str(max_type)+"."+str(max_arg)
  value = max_value
  print "which: ", which
  print "value: ", value

  img = cv2.imread(hist_dir+str(max_type)+"/"+str(max_arg)+"/area.jpg")
  match_hist(color, which, method, smoothing, True, hist_dir) # view histgram
  show_image(img, "best_match")
  return which, value, match_list

def succeed_result(res):
  f = open("succeed.txt", "a+")
  f.write(str(res[0]) + ": " + str(res[1])+"\n")
  f.close()

def fail_result(res, correct):
  f = open("fail.txt", "a+")
  f.write("correct: " + str(correct) + " -> " + str(res[0]) + ": " + str(res[1])+"\n")
  f.close()

