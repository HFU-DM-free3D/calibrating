import cv2
import os

def filter_brightness(image, threshold_value):
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_image, threshold_value, 255, cv2.THRESH_BINARY_INV)
    mask_3channel = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    filtered_image = cv2.bitwise_and(image, mask_3channel)
    return filtered_image

def process_images_in_folder(folder_path, threshold_value):
    # Iterate over all files in the folder
    for filename in os.listdir(folder_path):
        if filename.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff')):
            # Construct full file path
            file_path = os.path.join(folder_path, filename)

            # Read the image
            print("line35 file path: ", file_path)
            image = cv2.imread(file_path)
            filtered_image = filter_brightness(image, threshold_value)
    #        cv2.imshow('Filtered Image', filtered_image)
    #        cv2.waitKey(1)
    #cv2.destroyAllWindows()

            if image is not None:
               # Apply the filter
               filtered_image = filter_brightness(image, threshold_value)

               # Save the filtered image (overwrite the original image)
               cv2.imwrite(file_path, filtered_image)
               print(f"Processed and saved: {filename}")
            else:
               print(f"Error: Unable to load image {filename}")

# Define the folder path and threshold value
folder_path = 'F:/Studium/Master/Semester_3/Forsch/Recordings/a2_aufteilung/standing2/charuco/jan/S2/depth/'
threshold_value = 11

# Process all images in the folder
process_images_in_folder(folder_path, threshold_value)