import serial
import time
import numpy as np
import matplotlib.pyplot as plt
from script import parse_hex_data, plot_heatmap 

def parse_hex_data(hex_data):
    """
    Parse the hex data into a 2D NumPy array of 16-bit integers.
    
    Parameters:
        hex_data (str): Multiline string with hexadecimal values.
    
    Returns:
        numpy.ndarray: 2D array of 16-bit integers.
    """
    # Split the data into lines
    lines = hex_data.strip().split('\n')
    lines = [line.strip() for line in hex_data.strip().split('\n')]
    
    # Parse each line
    parsed_data = []
    for line in lines:
        # Split the line into 4-character chunks (16-bit hex values)
        row = [int(line[i:i+4], 16) for i in range(0, len(line), 4)]
        parsed_data.append(row)
    
    # Convert to a NumPy array
    return np.array(parsed_data, dtype=np.uint16)

def plot_heatmap(data, title="Heatmap", cmap="viridis", output_file=None):
    """
    Plots a heatmap from a 2D array of integers.
    
    Parameters:
        data (numpy array): 2D array of 16-bit integers.
        title (str): Title of the heatmap.
        cmap (str): Colormap for the heatmap.
        output_file (str): Path to save the heatmap image. If None, it shows the plot.
    """
    plt.figure(figsize=(10, 8))
    plt.imshow(data, cmap=cmap, aspect='auto')
    plt.colorbar(label='Intensity')
    plt.title(title)
    plt.xlabel('Columns')
    plt.ylabel('Rows')

    # Save or show the heatmap
    if output_file:
        plt.savefig(output_file, dpi=300, bbox_inches='tight')
        print(f"Heatmap saved to {output_file}")
    else:
        plt.show()

def read_from_sensor(port, baud_rate, char_to_send):
    try:
        ser = serial.Serial(port, baud_rate, timeout=1)
        time.sleep(2)  

        print(f"Sending character: {char_to_send}")
        ser.write(char_to_send.encode())  

        print("Received:")
        data = []
        
        while True:
            line = ser.readline().decode('utf-8').strip()
            if not line:  
                break
            data.append(line)

        print(data[0])
        data = data[1:]
        data = '\n'.join(data)
        # print(data)

        ser.close()
        return data
    except serial.SerialException as e:
        print(f"Serial Error: {e}")
        return ""
    except Exception as e:
        print(f"Unexpected Error: {e}")
        return ""

if __name__ == "__main__":
    serial_port = "/dev/ttyACM0"  
    baud_rate = 115200            
    character = "A"               

    raw_data = read_from_sensor(serial_port, baud_rate, character)
    parsed_data = parse_hex_data(raw_data)
    plot_heatmap(parsed_data)
