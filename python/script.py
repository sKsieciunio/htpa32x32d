import numpy as np
import matplotlib.pyplot as plt
import serial


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
        row = [int(line[i:i + 4], 16) for i in range(0, len(line), 4)]
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
        output_file (str): Path to save the heatmap image. If None,
        it shows the plot.
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


if __name__ == "__main__":
    ser = serial.Serial(
        port='COM7',
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE
    )

    try:
        if not ser.is_open:
            ser.open()

        print(f"Connected to {ser.name}")

        while True:
            char_to_send = input("Press Enter...")

            if char_to_send:
                char_to_send = char_to_send[0]
            else:
                char_to_send = ' '

            ser.write(char_to_send.encode())
            print(f"Sent: {repr(char_to_send)}")

            print("Receiving response:")
            data = []

            for i in range(32):
                line = ser.readline().decode('utf-8').strip()

                if not line:
                    break

                data.append(line)

            data = '\n'.join(data)
            print(data)

            parsed_data = parse_hex_data(data)
            parsed_data = (parsed_data / 10) - 273.15

            plot_heatmap(parsed_data, title="Parsed heatmap")

    except serial.SerialException as e:
        print(f"Error: {str(e)}")
    finally:
        if ser.is_open:
            ser.close()
            print("Serial connection closed")
