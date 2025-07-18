#!/usr/bin/env python3

import rospy
from robot_msgs.msg import encoder
from std_msgs.msg import Int32  # New message type for counter
import tkinter as tk
from tkinter import ttk
import threading

class EncoderGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS Encoder Monitor")
        
        # Initialize ROS in main thread
        rospy.init_node('encoder_gui', anonymous=False)
        
        # Create counter publisher
        self.counter_pub = rospy.Publisher('/gui/counter', Int32, queue_size=10)
        self.counter_value = 0
        
        # Create GUI widgets
        self.create_widgets()
        
        # Setup ROS subscriber
        self.sub = rospy.Subscriber('/sensor/encoder', encoder, self.encoder_callback)
        
        # Start ROS spinner in background thread
        self.ros_thread = threading.Thread(target=rospy.spin)
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Setup window close handler
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
    
    def create_widgets(self):
        """Create and arrange all GUI widgets"""
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # Encoder value display
        value_frame = ttk.LabelFrame(main_frame, text="Encoder Values", padding="10")
        value_frame.pack(fill=tk.X, pady=5)
        
        # Create labels for each encoder
        ttk.Label(value_frame, text="Encoder A:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=2)
        self.val_a = ttk.Label(value_frame, text="0", width=10)
        self.val_a.grid(row=0, column=1, padx=5, pady=2)
        
        ttk.Label(value_frame, text="Encoder B:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=2)
        self.val_b = ttk.Label(value_frame, text="0", width=10)
        self.val_b.grid(row=1, column=1, padx=5, pady=2)
        
        ttk.Label(value_frame, text="Encoder C:").grid(row=2, column=0, sticky=tk.W, padx=5, pady=2)
        self.val_c = ttk.Label(value_frame, text="0", width=10)
        self.val_c.grid(row=2, column=1, padx=5, pady=2)
        
        ttk.Label(value_frame, text="Encoder X:").grid(row=3, column=0, sticky=tk.W, padx=5, pady=2)
        self.val_x = ttk.Label(value_frame, text="0", width=10)
        self.val_x.grid(row=3, column=1, padx=5, pady=2)
        
        ttk.Label(value_frame, text="Encoder Y:").grid(row=4, column=0, sticky=tk.W, padx=5, pady=2)
        self.val_y = ttk.Label(value_frame, text="0", width=10)
        self.val_y.grid(row=4, column=1, padx=5, pady=2)
        
        # Button frame
        button_frame = ttk.Frame(main_frame)
        button_frame.pack(fill=tk.X, pady=10)
        
        # New counter button
        ttk.Button(button_frame, 
                 text="Publish Counter", 
                 command=self.publish_counter).pack(side=tk.LEFT, padx=5)
        
        # Counter value display
        self.counter_label = ttk.Label(button_frame, text="Counter: 0")
        self.counter_label.pack(side=tk.LEFT, padx=5)
        
        # Quit button
        ttk.Button(button_frame, text="Quit", command=self.on_close).pack(side=tk.RIGHT)
    
    def publish_counter(self):
        """Publish an incrementing counter"""
        self.counter_value += 1
        counter_msg = Int32()
        counter_msg.data = self.counter_value
        self.counter_pub.publish(counter_msg)
        self.counter_label.config(text=f"Counter: {self.counter_value}")
    
    def encoder_callback(self, msg):
        """Callback for ROS subscriber"""
        # Schedule GUI update in main thread
        self.root.after(0, self.update_gui, msg.enc_a, msg.enc_b, msg.enc_c, msg.enc_x, msg.enc_y)
    
    def update_gui(self, a, b, c, x, y):
        """Update the GUI with new encoder values"""
        self.val_a.config(text=str(a))
        self.val_b.config(text=str(b))
        self.val_c.config(text=str(c))
        self.val_x.config(text=str(x))
        self.val_y.config(text=str(y))
    
    def on_close(self):
        """Clean up before closing"""
        rospy.signal_shutdown("GUI closed")
        self.root.destroy()

def main():
    root = tk.Tk()
    
    # Set window size and position
    window_width = 350  # Slightly wider for new button
    window_height = 250
    screen_width = root.winfo_screenwidth()
    screen_height = root.winfo_screenheight()
    center_x = int(screen_width/2 - window_width/2)
    center_y = int(screen_height/2 - window_height/2)
    root.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')
    
    # Create and run application
    app = EncoderGUI(root)
    
    # Combine ROS and Tkinter event loops
    def spin():
        if not rospy.is_shutdown():
            root.after(100, spin)
        else:
            root.quit()
    
    spin()  # Start the combined event loop
    root.mainloop()

if __name__ == '__main__':
    main()

# #!/usr/bin/env python3

# import rospy
# from robot_msgs.msg import encoder
# import tkinter as tk
# from tkinter import ttk
# import threading

# class EncoderGUI:
#     def __init__(self, root):
#         self.root = root
#         self.root.title("ROS Encoder Monitor")
        
#         # Initialize ROS in main thread
#         rospy.init_node('encoder_gui', anonymous=False)
        
#         # Create GUI widgets
#         self.create_widgets()
        
#         # Setup ROS subscriber
#         self.sub = rospy.Subscriber('/sensor/encoder', encoder, self.encoder_callback)
        
#         # Start ROS spinner in background thread
#         self.ros_thread = threading.Thread(target=rospy.spin)
#         self.ros_thread.daemon = True
#         self.ros_thread.start()
        
#         # Setup window close handler
#         self.root.protocol("WM_DELETE_WINDOW", self.on_close)
    
#     def create_widgets(self):
#         """Create and arrange all GUI widgets"""
#         main_frame = ttk.Frame(self.root, padding="10")
#         main_frame.pack(fill=tk.BOTH, expand=True)
        
#         # Encoder value display
#         value_frame = ttk.LabelFrame(main_frame, text="Encoder Values", padding="10")
#         value_frame.pack(fill=tk.X, pady=5)
        
#         # Create labels for each encoder
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
        
#         # Quit button
#         button_frame = ttk.Frame(main_frame)
#         button_frame.pack(fill=tk.X, pady=10)
#         ttk.Button(button_frame, text="Quit", command=self.on_close).pack(side=tk.RIGHT)
    
#     def encoder_callback(self, msg):
#         """Callback for ROS subscriber"""
#         # Schedule GUI update in main thread
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
    
#     # Set window size and position
#     window_width = 300
#     window_height = 250
#     screen_width = root.winfo_screenwidth()
#     screen_height = root.winfo_screenheight()
#     center_x = int(screen_width/2 - window_width/2)
#     center_y = int(screen_height/2 - window_height/2)
#     root.geometry(f'{window_width}x{window_height}+{center_x}+{center_y}')
    
#     # Create and run application
#     app = EncoderGUI(root)
    
#     # Combine ROS and Tkinter event loops
#     def spin():
#         if not rospy.is_shutdown():
#             root.after(100, spin)
#         else:
#             root.quit()
    
#     spin()  # Start the combined event loop
#     root.mainloop()

# if __name__ == '__main__':
#     main()