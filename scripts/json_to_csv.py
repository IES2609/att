import json
import csv

def json_to_csv(input_filename, output_filename):
    # Load the JSON data
    with open(input_filename, 'r') as f:
        data = json.load(f)

    # Define the headers for the CSV
    headers = [
        'ax1_x', 'ax1_y', 'ax1_z',
        'ax2_x', 'ax2_y', 'ax2_z',
        'gyr_x', 'gyr_y', 'gyr_z'
    ]

    # Open the CSV file for writing
    with open(output_filename, 'w', newline='') as f:
        writer = csv.writer(f)
        
        # Write the header row
        writer.writerow(headers)

        # Iterate through each sample
        for sample in data.get('samples', []):
            decoded = sample.get('decoded', {})
            
            # Extract values, defaulting to None if missing
            ax1 = decoded.get('ax1', {})
            ax2 = decoded.get('ax2', {})
            gyr = decoded.get('gyr', {})

            row = [
                ax1.get('x'), ax1.get('y'), ax1.get('z'),
                ax2.get('x'), ax2.get('y'), ax2.get('z'),
                gyr.get('x'), gyr.get('y'), gyr.get('z')
            ]
            
            writer.writerow(row)

    print(f"Successfully converted {input_filename} to {output_filename}")

# Run the script
if __name__ == "__main__":
    json_to_csv('decoded.json', 'sorted.csv')