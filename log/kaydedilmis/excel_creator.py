import re
import pandas as pd

def parse_line(line):
    match = re.search(r'world (\d+)', line)
    if match:
        return int(match.group(1))
    return None

def process_txt(txt_path):
    data = {'world no': [], 'status': [], 'total dist': [], 'avg exec time': []}
    with open(txt_path, 'r') as file:
        lines = file.readlines()

    for i in range(len(lines)):
        line = lines[i].strip()

        world_number = parse_line(line)
        if world_number is not None:
            data['world no'].append(world_number)

            if "Collision occured" in lines[i + 1]:
                data['status'].append('Collision')
                data['total dist'].append(None)
                avg_exec_time_match = re.search(r'Avg execution time per cycle is: (\d+\.\d+)', lines[i + 1])
                if avg_exec_time_match:
                    data['avg exec time'].append(float(avg_exec_time_match.group(1)))
                else:
                    data['avg exec time'].append(None)
            elif "Goal Reached!" in lines[i + 1]:
                data['status'].append('Goal Reached')
                total_dist_match = re.search(r'Total distance traveled is: (\d+\.\d+)', lines[i + 1])
                if total_dist_match:
                    data['total dist'].append(float(total_dist_match.group(1)))
                else:
                    data['total dist'].append(None)

                avg_exec_time_match = re.search(r'Avg execution time per cycle is: (\d+\.\d+)', lines[i + 1])
                if avg_exec_time_match:
                    data['avg exec time'].append(float(avg_exec_time_match.group(1)))
                else:
                    data['avg exec time'].append(None)

    df = pd.DataFrame(data)
    df.to_excel('output.xlsx', index=False)

# Kullanım örneği
process_txt('nonpredictive_emergency.txt')

