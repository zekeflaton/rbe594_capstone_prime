
def write_line_to_file(filepath, array, open_mode="a"):
    with open(filepath, open_mode) as f:
        f.write(",".join(array))
        f.write("\n")
