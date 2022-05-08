import json
import bagpy
import pandas as pd

def convert_to_velocites(joint_positions):
    raise NotImplementedError

def read_bag_file_to_pd(bag_file, topic='/wx250s/commands/joint_group'):
    bag = bagpy.bagreader(bag_file)
    message_by_topic = bag.message_by_topic(topic)
    message_by_topic = pd.read_csv(message_by_topic)
    return message_by_topic

def get_joint_positions_(bag_file):
    message_by_topic = read_bag_file_to_pd(bag_file)
    columns = message_by_topic.columns

    joint_columns = message_by_topic.loc[:, columns[2:]]
    joint_positions = joint_columns.values.tolist()
    return np.array(joint_positions)

def joint_angles_to_json(joint_positions, file, ex=''):
    
    print('num joint commands: ', len(joint_positions))
    name, ext = os.path.splitext(file)
    with open(name + ex + '.json', 'w') as f:
        try:
            json.dump(joint_positions, f)
        except TypeError:
            json.dump(joint_positions.tolist(), f)