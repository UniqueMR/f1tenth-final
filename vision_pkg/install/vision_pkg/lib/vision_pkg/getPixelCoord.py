import cv2

# Callback function to catch the mouse events
def click_event(event, x, y, flags, params):
    # Check if the event was a left click
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Pixel Coordinates: x = {x}, y = {y}")

        # You can also mark the point and display it on the image
        # Uncomment the lines below if you want to see the point on the image
        # cv2.circle(img, (x, y), 5, (255, 0, 0), -1)
        # cv2.imshow("image", img)

# Load the image
img = cv2.imread("./resource/cone_x40cm.png")
# Set the window to the same resolution as the image
cv2.namedWindow('image', cv2.WINDOW_NORMAL)
cv2.resizeWindow('image', 960, 540)

# Set the callback function for any mouse event
cv2.setMouseCallback('image', click_event)

cv2.imshow('image', img)

# Wait indefinitely until a key is pressed
cv2.waitKey(0)

# Destroy all windows
cv2.destroyAllWindows()
