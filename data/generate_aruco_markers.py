import cv2
import os

def main():
    # We are using the 5X5_50 dictionary as specified in aruco_detector.py
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_50)
    
    # Grid shelves mapped to their new marker IDs
    # 1A = 0, 1B = 1, 2A = 2, 2B = 3, 3A = 4, 3B = 5
    shelves = ["1A", "1B", "2A", "2B", "3A", "3B"]
    
    output_dir = os.path.dirname(os.path.abspath(__file__))
    
    for marker_id, shelf in enumerate(shelves):
        # Generate a 400x400 pixel image of the marker
        marker_image = cv2.aruco.generateImageMarker(dictionary, marker_id, 400)
        
        # Add a white border so it prints cleanly and is easier to detect
        marker_image = cv2.copyMakeBorder(
            marker_image, 
            40, 40, 40, 40, 
            cv2.BORDER_CONSTANT, 
            value=[255, 255, 255]
        )
        
        filename = f"aruco_marker_{marker_id}_shelf_{shelf}.png"
        filepath = os.path.join(output_dir, filename)
        
        cv2.imwrite(filepath, marker_image)
        print(f"Generated {filename} for Shelf {shelf} (ID {marker_id})")

if __name__ == "__main__":
    main()
