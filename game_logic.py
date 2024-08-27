import rospy
from your_package.msg import TurtleHealth, TurtleAttackStatus
from movement_update.msg import TurtlePosition
from std_msgs.msg import String

class Turtle:
    def __init__(self, name, x=0, y=0, radius=1):
        self.name = name
        self.health = 100
        self.attacks_left = 10
        self.x = x
        self.y = y
        self.radius = radius  #turtle's radius

    def apply_damage(self, damage_amount):
        self.health -= damage_amount
        if self.health < 0:
            self.health = 0



class GameEngine:
    def __init__(self):
        rospy.init_node('game_engine')

        self.max_turtles = 7
        self.turtles = []
        self.game_over = False
        self.winner = None
        self.collision_damage = 10  # damage amount for each collision

        # Subscribe to turtle health and attack status
        for i in range(self.max_turtles):
            rospy.Subscriber(f'/turtle{i+1}/health', TurtleHealth, self.turtle_health_callback, i) # '/health' and 'TurtleHealth' to be modified with the actual topic name and the message type that is published on the topic to be subscribed to
            rospy.Subscriber(f'/turtle{i+1}/attack_status', TurtleAttackStatus, self.turtle_attack_status_callback, i) # '/attack_status' and TurtleAttackStatus to be modifed """
            rospy.Subscriber(f'/turtle{i+1}/position', TurtlePosition, self.turtle_position_callback, i) # 'pos' and 'turtlepos' """

        
        pub = rospy.Publisher('game_logic', String, queue_size=10)
        self.publish_game_start_message()


        rospy.spin()  # for the node to keep running and processing incoming messages throughout the game

    def publish_game_start_message(self):
        msg = String()
        msg.data = "Turtles' Battle Begin"
        self.pub.publish(msg)


    def create_turtle(self, name):
        if len(self.turtles) < self.max_turtles:
            new_turtle = Turtle(name)
            self.turtles.append(new_turtle)
            rospy.loginfo(f"Turtle {name} has joined the game!")

    def turtle_health_callback(self, health_msg, turtle_index):
        if 0 <= turtle_index < len(self.turtles):
            self.turtles[turtle_index].health = health_msg.health
            self.check_game_over()
        else:
            rospy.logwarn(f"Received health update for unknown turtle index {turtle_index}")

    def turtle_attack_status_callback(self, attack_status_msg, turtle_index):
        if 0 <= turtle_index < len(self.turtles):
            self.turtles[turtle_index].attacks_left = attack_status_msg.attacks_left
            self.detect_and_handle_collisions()
        else:
            rospy.logwarn(f"Received attack status update for unknown turtle index {turtle_index}")

    def turtle_position_callback(self, position_msg, turtle_index):
        if 0 <= turtle_index < len(self.turtles):
            self.turtles[turtle_index].x = position_msg.x
            self.turtles[turtle_index].y = position_msg.y
            self.detect_and_handle_collisions()
        else:
            rospy.logwarn(f"Received position update for unknown turtle index {turtle_index}")

    
    def check_collision(self, turtle1, turtle2):
     # Calculate the distance between the centers of the two turtles
     distance = ((turtle1.x - turtle2.x) ** 2 + (turtle1.y - turtle2.y) ** 2) ** 0.5
    
     # Check if the distance is less than the sum of their radii
     return distance < (turtle1.radius + turtle2.radius)


    def detect_and_handle_collisions(self):
     # First pass: detect collisions
     for i in range(len(self.turtles)):
        for j in range(i + 1, len(self.turtles)):
            if self.check_collision(self.turtles[i], self.turtles[j]):
                self.collisions.append((i, j))

    # Second pass: handle the detected collisions
     for i, j in self.collisions:
        self.apply_damage(self.turtles[i], self.turtles[j])
        self.apply_damage(self.turtles[j], self.turtles[i])

    def apply_damage(self, attacker, defender):
        if attacker.attacks_left > 0:
            defender.apply_damage(self.collision_damage)
            attacker.attacks_left -= 1
            rospy.loginfo(f"{attacker.name} hit {defender.name} for {self.collision_damage} damage!")


    def check_game_over(self):
        if all(turtle.attacks_left == 0 for turtle in self.turtles):
            self.game_over = True
            winner_index = max(range(len(self.turtles)), key=lambda i: self.turtles[i].health)
            self.winner = self.turtles[winner_index].name
            rospy.loginfo(f"Game Over! Winner: {self.winner}")

    def publish_game_over_message(self):
        msg = String()
        msg.data = f"Game Over! Winner: {self.winner}"
        self.pub.publish(msg)