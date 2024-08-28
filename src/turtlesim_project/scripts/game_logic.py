#!/usr/bin/env python3

import rospy
from turtlesim.msg import Pose
from turtlesim.srv import Kill
from std_msgs.msg import String

class Turtle:
    def __init__(self, name, x=0, y=0, radius=1):
        self.name = name
        self.x = x
        self.y = y
        self.health = 100
        self.attacks_left = 10
        self.radius = radius  #turtle's radius

    def apply_damage(self):
        self.health -= 50
        if self.health < 0:
            self.health = 0



class GameEngine:
    def __init__(self):
        rospy.init_node('game_engine')

        self.numTurtles = 1
        self.max_turtles = 7
        self.turtles = []
        self.game_over = False
        self.winner = None
        self.collision_damage = 50  # damage amount for each collision
        self.pose_subscribers = []

        self.spawns_subscriber = rospy.Subscriber('/spawns', String, self.create_turtle)
        self.attacks_subscriber = rospy.Subscriber('/attacks', String, self.handle_attack)




        self.pose = Pose()
        self.rate = rospy.Rate(10)
        
        self.publish_game_start_message()
        self.add_default_turtle()

        for i in range( self.max_turtles ):
            rospy.Subscriber(f"/turtle{i+1}/pose", Pose, self.update_poses, callback_args=i)
 
        rospy.spin()  # for the node to keep running and processing incoming messages throughout the game


    def publish_game_start_message(self):
        rospy.loginfo("\n\n-----\t-----\t----\nTurtles' Battle Begin\n")

    def add_default_turtle(self):
        default_name = "turtle1"
        default_turtle = Turtle(name=default_name, x=0, y=0)
        self.turtles.append(default_turtle)
        rospy.loginfo(f"Added default turtle: {default_turtle.name}")


    def create_turtle(self, msg):

        if msg.data :
            if len(self.turtles) > 7:
                rospy.logwarn("Maximum number of turtles reached can't add more!")
                return
            
            # Create a new turtle from the message data
            new_turtle = Turtle(name=msg.data, x=0, y=0)
            self.turtles.append(new_turtle)
            rospy.loginfo(f"Added new turtle: {new_turtle.name}")
            self.numTurtles+=1

   
    def update_poses(self, pose, turtle_index):

        if turtle_index < len(self.turtles):
            self.turtles[turtle_index].x = round(pose.x, 4)
            self.turtles[turtle_index].y = round(pose.y, 4)

    
    def handle_attack(self, msg):
        if msg.data: 
            attacker_name = msg.data
            # Find the attacker turtle by name
            for i in range( len(self.turtles) ):
                if self.turtles[i].name == attacker_name:
                    attacker = self.turtles[i]
                    break 

            # Check distance and apply damage
            for turtle in self.turtles:
                if turtle.name != attacker_name:  # Skip the attacker itself
                    # rospy.loginfo(f"attacker.x :{attacker.x} turtle.x: {turtle.x:} \n attacker.y : {attacker.y}  turtle.y:{turtle.y}")
                    distance = ((attacker.x - turtle.x) ** 2 + (attacker.y - turtle.y) ** 2) ** 0.5
                    # rospy.loginfo(f"Distance: {distance}")
                    if distance < (attacker.radius + turtle.radius):  # Check if within attack range
                        self.apply_damage(attacker, turtle)
                        

    def apply_damage(self, attacker, defender):
        if attacker.attacks_left > 0:
            defender.apply_damage()
            attacker.attacks_left -= 1
            rospy.loginfo(f"\n{attacker.name} attacked {defender.name}!")
            rospy.loginfo(f"{defender.name} health now: {defender.health}\n\n")
        else:
            rospy.loginfo(f"{attacker.name} has no attacks left!\n\n")

        if defender.health == 0:
            defender.x, defender.y = 100,100
            self.remove_turtle(defender.name)

        if all(turtle.attacks_left == 0 for turtle in self.turtles) or self.numTurtles < 2:
            winner_index = max(range(len(self.turtles)), key=lambda i: self.turtles[i].health)
            self.winner = self.turtles[winner_index].name
            rospy.loginfo(f"Game Over! Winner: {self.winner}")
            rospy.signal_shutdown("THE GAME ENDED.")
            return
        
    def remove_turtle(self, turtle_name):
     rospy.wait_for_service('/kill')
     try:
         kill_turtle = rospy.ServiceProxy('/kill', Kill)
         kill_turtle(turtle_name)
         self.numTurtles-=1
         rospy.loginfo(f"Turtle {turtle_name} has been removed.")
     except rospy.ServiceException:
         pass
        #  rospy.logerr(f"Service call failed: {e}")





def main():
    game_engine = GameEngine()

if __name__ == '__main__':
    main()