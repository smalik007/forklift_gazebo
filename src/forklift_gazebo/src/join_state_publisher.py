import Tkinter as tk
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

def reset_model():
    print("reset")
    

class Slider:

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("GUI Slider")

        self.w1 = tk.Scale(self.root, label="Shift-right", length=500, from_=0, to=100, orient=tk.HORIZONTAL)
        self.w1.pack()
        self.w1.bind("<ButtonRelease-1>", self.updateValue_w1)

        self.w2 = tk.Scale(self.root, label="Shift-left", length=500, from_=0, to=100, orient=tk.HORIZONTAL)
        self.w2.pack()
        self.w2.bind("<ButtonRelease-1>", self.updateValue_w2)

        #self.b1 = tk.Button(text='Reset', command=reset_model).pack()

        self.root.mainloop()

    def updateValue_w1(self, event):
        print("w1:{}".format(self.w1.get()))
        self.w2.set(0)
        self.join_state_publish("shift", (self.w1.get()*0.1)/100)

    def updateValue_w2(self, event):
        print("w2:{}".format(self.w2.get()))
        self.w1.set(0)
        self.join_state_publish("shift", (self.w2.get()*-0.1)/100)
    
    def join_state_publish(self, joint_name, value):
        
        #print("pub:{}, value:{}".format(joint_name, value))

        pub = rospy.Publisher('joint_states', JointState, queue_size=10)
        rospy.init_node('joint_state_publisher')
        rate = rospy.Rate(10) # 10hz
        hello_str = JointState()
        hello_str.header = Header()
        hello_str.header.stamp = rospy.Time.now()
        hello_str.name = [joint_name]
        hello_str.position = [value]
        hello_str.velocity = [0]
        hello_str.effort = [0]
        pub.publish(hello_str)
        rate.sleep()


if __name__ == "__main__":
    slider = Slider()
    #slider.show_values()