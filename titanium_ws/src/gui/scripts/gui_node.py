#!/usr/bin/env python

import rospy
import tkinter as tk
from tkinter import ttk
from sensor_msgs.msg import Image as ROSImage  # Renamed to avoid conflict
from cv_bridge import CvBridge, CvBridgeError
import cv2
from PIL import Image, ImageTk
import threading
import numpy as np

class ROSCameraGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS Camera Feed")
        self.root.geometry("800x600")
        
        # ROS initialization
        rospy.init_node('camera_gui_node', anonymous=True)
        self.bridge = CvBridge()
        
        # Image variables
        self.current_image = None
        self.tk_image = None
        
        # Create GUI
        self.setup_gui()
        
        # ROS subscriber - using ROSImage instead of Image
        self.image_sub = rospy.Subscriber("/camera/image_raw", ROSImage, self.image_callback)
        
        # Start update thread
        self.running = True
        self.update_thread = threading.Thread(target=self.update_gui)
        self.update_thread.daemon = True
        self.update_thread.start()
        
    def setup_gui(self):
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Image display
        self.image_label = ttk.Label(main_frame)
        self.image_label.grid(row=0, column=0, columnspan=2, pady=10)
        
        # Controls frame
        controls_frame = ttk.Frame(main_frame)
        controls_frame.grid(row=1, column=0, columnspan=2, pady=10)
        
        # Topic selection
        ttk.Label(controls_frame, text="Camera Topic:").grid(row=0, column=0, padx=5)
        self.topic_var = tk.StringVar(value="/camera/raw_image")
        topic_entry = ttk.Entry(controls_frame, textvariable=self.topic_var, width=30)
        topic_entry.grid(row=0, column=1, padx=5)
        
        # Subscribe button
        subscribe_btn = ttk.Button(controls_frame, text="Subscribe", command=self.change_topic)
        subscribe_btn.grid(row=0, column=2, padx=5)
        
        # Status label
        self.status_var = tk.StringVar(value="Waiting for images...")
        status_label = ttk.Label(controls_frame, textvariable=self.status_var)
        status_label.grid(row=1, column=0, columnspan=3, pady=5)
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(0, weight=1)
        
    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.current_image = cv_image
            self.status_var.set(f"Image received: {cv_image.shape}")
        except CvBridgeError as e:
            self.status_var.set(f"Error: {str(e)}")
            print(f"CvBridge Error: {e}")
        except Exception as e:
            self.status_var.set(f"Error: {str(e)}")
            print(f"Error: {e}")
    
    def change_topic(self):
        new_topic = self.topic_var.get()
        try:
            # Unsubscribe from current topic
            self.image_sub.unregister()
            # Subscribe to new topic
            self.image_sub = rospy.Subscriber(new_topic, ROSImage, self.image_callback)
            self.status_var.set(f"Subscribed to: {new_topic}")
        except Exception as e:
            self.status_var.set(f"Error subscribing: {str(e)}")
    
    def update_gui(self):
        while self.running and not rospy.is_shutdown():
            if self.current_image is not None:
                try:
                    # Resize image for display
                    display_image = self.resize_image(self.current_image, 640, 480)
                    
                    # Convert to PIL format
                    image_rgb = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
                    pil_image = Image.fromarray(image_rgb)
                    
                    # Convert to Tkinter format
                    self.tk_image = ImageTk.PhotoImage(pil_image)
                    
                    # Update GUI in main thread
                    self.root.after(0, self.update_image_display)
                    
                except Exception as e:
                    print(f"GUI update error: {e}")
            
            rospy.sleep(0.033)  # ~30 FPS
    
    def update_image_display(self):
        if self.tk_image:
            self.image_label.configure(image=self.tk_image)
    
    def resize_image(self, image, max_width, max_height):
        height, width = image.shape[:2]
        
        # Calculate scaling factor
        scale = min(max_width/width, max_height/height)
        
        if scale < 1:
            new_width = int(width * scale)
            new_height = int(height * scale)
            return cv2.resize(image, (new_width, new_height))
        return image
    
    def on_closing(self):
        self.running = False
        if self.update_thread.is_alive():
            self.update_thread.join(timeout=1.0)
        self.root.destroy()
        rospy.signal_shutdown("GUI closed")

def main():
    root = tk.Tk()
    app = ROSCameraGUI(root)
    
    # Set close handler
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    
    try:
        root.mainloop()
    except KeyboardInterrupt:
        app.on_closing()

if __name__ == "__main__":
    main()






# #!/usr/bin/env python3

# import rospy
# from robot_msgs.msg import encoder
# from std_msgs.msg import Int32
# import tkinter as tk
# from tkinter import ttk
# import threading

# class EncoderGUI:
#     def __init__(self, root):
#         self.root = root
#         self.root.title("ROS Encoder Monitor")
        
#         rospy.init_node('encoder_gui', anonymous=False)
        
#         self.counter_pub = rospy.Publisher('/gui/counter', Int32, queue_size=10)
#         self.counter_value = 0
        
#         self.create_widgets()
        
#         self.sub = rospy.Subscriber('/sensor/encoder', encoder, self.encoder_callback)
        
#         self.ros_thread = threading.Thread(target=rospy.spin)
#         self.ros_thread.daemon = True
#         self.ros_thread.start()
        
#         self.root.protocol("WM_DELETE_WINDOW", self.on_close)
    
#     def create_widgets(self):
#         """Create and arrange all GUI widgets"""
#         main_frame = ttk.Frame(self.root, padding="10")
#         main_frame.pack(fill=tk.BOTH, expand=True)
        
#         value_frame = ttk.LabelFrame(main_frame, text="Encoder Values", padding="10")
#         value_frame.pack(fill=tk.X, pady=5)
        
#         ttk.Label(value_frame, text="Encoder A:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
#         self.val_a = ttk.Label(value_frame, text="0", width=10)
#         self.val_a.grid(row=0, column=1, padx=5, pady=2)
        
#         ttk.Label(value_frame, text="Encoder B:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
#         self.val_b = ttk.Label(value_frame, text="0", width=10)
#         self.val_b.grid(row=1, column=1, padx=5, pady=2)
        
#         ttk.Label(value_frame, text="Encoder C:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=2)
#         self.val_c = ttk.Label(value_frame, text="0", width=10)
#         self.val_c.grid(row=2, column=1, padx=5, pady=2)
        
#         ttk.Label(value_frame, text="Encoder X:").grid(row=3, column=0, sticky=tk.W, padx=5, pady=2)
#         self.val_x = ttk.Label(value_frame, text="0", width=10)
#         self.val_x.grid(row=3, column=1, padx=5, pady=2)
        
#         ttk.Label(value_frame, text="Encoder Y:").grid(row=4, column=0, sticky=tk.W, padx=5, pady=2)
#         self.val_y = ttk.Label(value_frame, text="0", width=10)
#         self.val_y.grid(row=4, column=1, padx=5, pady=2)
        
#         button_frame = ttk.Frame(main_frame)
#         button_frame.pack(fill=tk.X, pady=10)
        
#         ttk.Button(button_frame, 
#                  text="Publish Counter", 
#                  command=self.publish_counter).pack(side=tk.LEFT, padx=5)
        
#         self.counter_label = ttk.Label(button_frame, text="Counter: 0")
#         self.counter_label.pack(side=tk.LEFT, padx=5)
        
#         ttk.Button(button_frame, text="Quit", command=self.on_close).pack(side=tk.RIGHT)
    
#     def publish_counter(self):
#         """Publish an incrementing counter"""
#         self.counter_value += 1
#         counter_msg = Int32()
#         counter_msg.data = self.counter_value
#         self.counter_pub.publish(counter_msg)
#         self.counter_label.config(text=f"Counter: {self.counter_value}")
    
#     def encoder_callback(self, msg):
#         """Callback for ROS subscriber"""
#         self.root.after(0, self.update_gui, msg.enc_a, msg.enc_b, msg.enc_c, msg.enc_x, msg.enc_y)
    
#     def update_gui(self, a, b, c, x, y):
#         """Update the GUI with new encoder values"""
#         self.val_a.config(text=str(a))
#         self.val_b.config(text=str(b))
#         self.val_c.config(text=str(c))
#         self.val_x.config(text=str(x))
#         self.val_y.config(text=str(y))
    
#     def on_close(self):
#         """Clean up before closing"""
#         rospy.signal_shutdown("GUI closed")
#         self.root.destroy()

# def main():
#     root = tk.Tk()
    
#     window_width = 350
#     window_height = 250
#     screen_width = root.winfo_screenwidth()
#     screen_height = root.winfo_screenheight()
#     center_x = int(screen_width/2 - window_width/2)
#     center_y = int(screen_height/2 - window_height/2)
#     root.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')
    
#     app = EncoderGUI(root)
    
#     def spin():
#         if not rospy.is_shutdown():
#             root.after(100, spin)
#         else:
#             root.quit()
    
#     spin()
#     root.mainloop()

# if __name__ == '__main__':
#     main()