import json
import numpy as np
import os

os.chdir("/home/funkey/ME-559/College/Lab 5")

# Function to load the transformation matrix from JSON
def load_transform_matrix(matrix_file="transform_matrix.json"):
    with open(matrix_file, "r") as f:
        matrix = json.load(f)
    return np.array(matrix, dtype="float32")

# Function to modify the matrix to invert the Y-axis
def invert_y_in_matrix(matrix):
    y_inversion = np.array([
        [1, 0, 0],
        [0, -1, 0],  # Invert Y-axis
        [0, 0, 1]
    ], dtype="float32")
    modified_matrix = np.dot(y_inversion, matrix)
    return modified_matrix

# Function to save the modified matrix back to JSON
def save_transform_matrix(matrix, output_file="modified_transform_matrix.json"):
    matrix_list = matrix.tolist()
    with open(output_file, "w") as f:
        json.dump(matrix_list, f, indent=4)
    print(f"Modified transformation matrix saved to {output_file}")

# Main process
matrix_file = "transform_matrix.json"
homography_matrix = load_transform_matrix(matrix_file)
modified_homography_matrix = invert_y_in_matrix(homography_matrix)
save_transform_matrix(modified_homography_matrix)
