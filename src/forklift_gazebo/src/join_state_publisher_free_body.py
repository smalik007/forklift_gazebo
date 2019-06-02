import Tkinter as tk
import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

class Slider:

    def __init__(self):
        self.root = tk.Tk()
        self.root.title("GUI Slider")

        self.shift = tk.Scale(self.root, label="Shift", length=500, from_=0, to=100, orient=tk.HORIZONTAL)
        self.shift.pack()
        self.shift.bind("<ButtonRelease-1>", self.updateValue_shift)
        #self.shift.set(50)

        self.lift = tk.Scale(self.root, label="Lift", length=200, from_=100, to=0, orient=tk.VERTICAL)
        self.lift.pack()
        self.lift.bind("<ButtonRelease-1>", self.updateValue_lift)

        self.b1 = tk.Button(text='Reset', command=self.reset_model).pack()

        self.root.mainloop()

    def updateValue_shift(self, event):
        print("shift:{}".format(self.shift.get()))
        self.join_state_publish("shift", ((self.shift.get()-50)*0.15)/50)


    def updateValue_lift(self, event):
        print("lift:{}".format(self.lift.get()))
        self.join_state_publish("lift", (self.lift.get()*0.5)/100)

    def reset_model(self):
        print("reset")
        self.shift.set(50)
        self.lift.set(0)
        self.join_state_publish("shift", 0)
        self.join_state_publish("lift", 0)

    
    def join_state_publish(self, joint_name, value):
        
        print("pub:{}, value:{}".format(joint_name, value))

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