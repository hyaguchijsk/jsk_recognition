# yellow: [27 177 255] 
# green: [ 94 143  73]
def get_hsv_filter_mask(color_bgr, hsv_threshold_list):
  # h_min, h_max, s_min, s_max, v_min, v_max
  h_min, h_max, s_min, s_max, v_min, v_max = hsv_threshold_list
  color_hsv = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2HSV)
  # h filter
  h_img = color_hsv[:,:,0]
  h_mask = (h_min<=h_img) & (h_img<=h_max)
  # s filter
  s_img = color_hsv[:,:,1]
  s_mask = (s_min<=s_img) & (s_img<=s_max)
  # v filter
  v_img = color_hsv[:,:,2]
  v_mask = (v_min<=v_img) & (v_img<=v_max)
  is_point_mask = h_mask * s_mask * v_mask
  return dstack_mask(is_point_mask)

def get_erode_and_dilate_mask(color, n=10):
  is_point_mask = (erode_and_dilate_filter(color,n)!=0)
  return dstack_mask(is_point_mask)

def dilate_and_erode_filter(img, n=10):
  gray_img = img.copy() if len(img.shape)==2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  element = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
  dilate_img = cv2.dilate(gray_img, element, iterations=n)
  erode_img = cv2.erode(dilate_img, element, iterations=n)
  is_point_mask = (erode_img!=0)
  return erode_img

def erode_and_dilate_filter(img, n=10):
  gray_img = img.copy() if len(img.shape)==2 else cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
  element = cv2.getStructuringElement(cv2.MORPH_RECT,(3,3))
  erode_img = cv2.erode(gray_img, element, iterations=n)
  dilate_img = cv2.dilate(erode_img, element, iterations=n)
  is_point_mask = (dilate_img!=0)
  return dilate_img

def dstack_mask(mask_img):
  return np.dstack((mask_img,)*3)

