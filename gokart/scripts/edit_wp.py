import csv

def subtract_from_first_column(input_file, output_file, offset):
    modified_rows = []
    first_row = True  # Add a flag to track the first row
    
    # Open the input CSV file
    with open(input_file, 'r', newline='') as csvfile:
        reader = csv.reader(csvfile)
        
        # Read each row in the file
        for row in reader:
            if first_row:
                # Skip modification for the first row (likely headers)
                first_row = False
            else:
                # Subtract 2 from the first column (assuming the first column contains numeric data)
                if row:  # Check if the row is not empty
                    try:
                        row[0] = str(float(row[0]) - offset)
                    except ValueError:
                        # Handle the case where conversion to float fails (e.g., non-numeric data)
                        print(f"Skipping modification for non-numeric value: {row[0]}")
                    
            modified_rows.append(row)
    
    # Open the output CSV file and write the modified data
    with open(output_file, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(modified_rows)

offset = 2.5
# Example usage:
input_csv_file = '/home/autoware/gokart_ws/src/gokart-sensor/configs/pennovation_lotA/wp.csv'  # Update this path to your input CSV file
output_csv_file = f'/home/autoware/gokart_ws/src/gokart-sensor/configs/pennovation_lotA/wp_shifted_{str(offset)}.csv'  # Update this path to where you want to save the output CSV file

subtract_from_first_column(input_csv_file, output_csv_file, offset)
