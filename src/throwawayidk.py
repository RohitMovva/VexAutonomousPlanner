#!/usr/bin/env python3

def parse_data_file(input_filename):
    """Parse the input file and read all lines."""
    trajectory_points = []
    
    with open(input_filename, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:  # Skip empty lines
                continue
            
            # Split the line and convert to floats
            elements = line.split()
            values = [float(x) for x in elements]
            
            trajectory_points.append(values)
    
    return trajectory_points


def format_vector_data(data):
    """Format a list of lists into C++ vector initialization syntax."""
    lines = []
    lines.append("{")
    
    for i, point in enumerate(data):
        # Format each point as {val1, val2, val3, ...}
        point_str = "    {" + ", ".join(str(val) for val in point) + "}"
        
        # Add comma if not the last element
        if i < len(data) - 1:
            point_str += ","
        
        lines.append(point_str)
    
    lines.append("}")
    return "\n".join(lines)


def create_header_file(trajectory_points, output_filename):
    """Generate the C++ header file."""
    
    trajectory_data = format_vector_data(trajectory_points)
    
    header_content = f"""#ifndef ROUTES_H
#define ROUTES_H

#include <vector>

std::vector<std::vector<double>> trajectory_points = 
{trajectory_data};

#endif
"""
    
    with open(output_filename, 'w') as f:
        f.write(header_content)
    
    print(f"Header file '{output_filename}' created successfully!")
    print(f"  - Total trajectory points: {len(trajectory_points)}")


def main():
    import sys
    
    if len(sys.argv) < 2:
        print("Usage: python script.py <input_file> [output_file]")
        print("Example: python script.py data.txt routes.h")
        sys.exit(1)
    
    input_file = sys.argv[1]
    output_file = sys.argv[2] if len(sys.argv) > 2 else "routes.h"
    
    # Parse the input file
    trajectory_points = parse_data_file(input_file)
    
    # Create the header file
    create_header_file(trajectory_points, output_file)


if __name__ == "__main__":
    main()