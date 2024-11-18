import yaml
# --------------------------- config import ---------------------------------------
with open("config.yml", "r") as ymlfile:
    config = yaml.safe_load(ymlfile)

# ---------------------------- PARAMETERS ------------------------------------------

bounding_matrix = config["bounding_matrix"]

motion_x = 160
motion_y = 550

#0: [50, 70, 0, 100]
#1: [150, 200, 500, 600]
#2: [90, 140, 200, 400]

target_out_bounds = 0

for i in range(0, len(bounding_matrix)):
    if not bounding_matrix[i][0] <= motion_x <= bounding_matrix[i][1] and not bounding_matrix[i][2] <= motion_y <= bounding_matrix[i][3]:
        target_out_bounds += 1

if target_out_bounds == len(bounding_matrix):
    print(motion_x, motion_y)
