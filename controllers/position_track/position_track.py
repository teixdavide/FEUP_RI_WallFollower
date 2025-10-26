from controller import Supervisor
import csv

supervisor = Supervisor()

timestep = int(supervisor.getBasicTimeStep())

root = supervisor.getRoot()
children_field = root.getField("children")

robot_node = None

for i in range(children_field.getCount()):
    child = children_field.getMFNode(i)
    if child.getField("name") is not None:
        name_field = child.getField("name")
        if name_field.getSFString() == "my_robot":
            robot_node = child

translation_field = robot_node.getField("translation")

with open('robot_position.csv', mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['time (s)', 'x', 'y'])

    time = 0.0
    while supervisor.step(timestep) != -1:
        pos = translation_field.getSFVec3f()

        x, y = pos[0], pos[2]

        writer.writerow([time / 1000.0, x, y])
        file.flush()

        time += timestep