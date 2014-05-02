def test_bounding(color, mask):
  res = color.copy()
  x,y,w,h = cv2.boundingRect(mask2points(mask))
  cv2.rectangle(res,(x,y),(x+w,y+h),(0,255,0),2)
  show_image(res, "bounding")

def test_rotated_rectangle(color, mask):
  res = color.copy()
  rect = cv2.minAreaRect(mask2points(mask))
  box = cv2.cv.BoxPoints(rect)
  box = np.int0(box)
  cv2.drawContours(res,[box],0,(0,0,255),2)
  show_image(res, "rotated_rectangle")

def test_circum_circle(color, mask):
  res = color.copy()
  (x,y),radius = cv2.minEnclosingCircle(mask2points(mask))
  center = (int(x),int(y))
  radius = int(radius)
  cv2.circle(res,center,radius,(0,255,0),2)
  show_image(res, "circum_circle")

def test_ellipse(color, mask):
  res = color.copy()
  ellipse_box = cv2.fitEllipse(mask2points(mask))
  # pick_point = tuple([int(i) for i in ellipse_box[0]]) # center
  cv2.ellipse(res, ellipse_box, (0,255,0), thickness=3)
  show_image(res, "ellipse")

# not complete
def test_ellipse_box(color, mask):
  res = color.copy()
  rotated_box = get_rotated_rectangle(mask)
  cv2.ellipse(res, rotated_box, (0,255,0), -1)
  show_image(res, "ellipse_box")

def test_approx_poly(color, mask):
  res = color.copy()
  points = mask2points(mask)
  approx = cv2.approxPolyDP(points, 0.1*cv2.arcLength(points,True),True)
  return approx
  show_image(res, "apploxy") 

# not complete
def test_convex_hull(color, mask):
  res = color.copy()
  hull = cv2.convexHull(mask2points(mask),returnPoints = False)
  points = cv2.convexHull(mask2points(mask),returnPoints = True)

  cv2.drawContours(res,points,0,(0,0,255),2)
  show_image(res, "hull") 
  return points

def test_fill_poly(color, mask):
  res = color.copy()
  rotated_box = get_rotated_rectangle(mask)
  if rotated_box.any():
    p1, p2, p3, p4 = rotated_box
  else:
    return 0

  cv2.fillConvexPoly(res, rotated_box, (0,0,0))

  show_image(res, "poly")
