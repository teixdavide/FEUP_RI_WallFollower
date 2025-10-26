import csv
from controller import Supervisor

supervisor = Supervisor()
timestep = int(supervisor.getBasicTimeStep())

root = supervisor.getRoot()
children_field = root.getField("children")

follower_node = None
myrobot_node = None

for i in range(children_field.getCount()):
    child = children_field.getMFNode(i)
    if child.getField("name") is not None:
        name_field = child.getField("name")
        if name_field.getSFString() == "my_robot":
            myrobot_node = child
        elif name_field.getSFString() == "follower":
            follower_node = child

follower_translation = follower_node.getField("translation")
myrobot_translation = myrobot_node.getField("translation")

with open('robot_positions.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['time (s)', 
                     'follower_x', 'follower_y', 
                     'myrobot_x', 'myrobot_y'])

    time = 0.0
    while supervisor.step(timestep) != -1:
        follower_pos = follower_translation.getSFVec3f()
        myrobot_pos = myrobot_translation.getSFVec3f()

        follower_x, follower_y = follower_pos[0], follower_pos[1]
        myrobot_x, myrobot_y = myrobot_pos[0], myrobot_pos[1]

        writer.writerow([time / 1000.0, 
                         follower_x, follower_y, 
                         myrobot_x, myrobot_y])
        file.flush()

        time += timestep
