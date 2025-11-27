from math import atan2, asin, sqrt

M_PI=3.1415926535

class Logger:
    def __init__(self, filename, headers=["e", "e_dot", "e_int", "stamp"]):
        # Initialize logger with filename and headers for CSV
        self.filename = filename

        with open(self.filename, 'w') as file:
            header_str=""

            for header in headers:
                header_str+=header
                header_str+=", "
            
            header_str+="\n"
            
            file.write(header_str)


    def log_values(self, values_list):
        # Append a list of values as a CSV row to the log file
        with open(self.filename, 'a') as file:
            vals_str=""

            for val in values_list:
                vals_str += str(val)
                vals_str += ", "

            vals_str+="\n"

            file.write(vals_str)
            

    def save_log(self):
        # Placeholder for saving log if needed
        pass

class FileReader:
    def __init__(self, filename):
        # Initialize file reader with filename
        self.filename = filename
        
        
    def read_file(self):
        # Read CSV file and return headers and data table
        read_headers=False

        table=[]
        headers=[]
        with open(self.filename, 'r') as file:
            # Skip the header line

            if not read_headers:
                for line in file:
                    values=line.strip().split(',')

                    for val in values:
                        if val=='':
                            break
                        headers.append(val.strip())

                    read_headers=True
                    break
            
            next(file)
            
            # Read each line and extract values
            for line in file:
                values = line.strip().split(',')
                
                row=[]                
                
                for val in values:
                    if val=='':
                        break
                    row.append(float(val.strip()))

                table.append(row)
        
        return headers, table


def euler_from_quaternion(quat):
    """
    Convert quaternion (w in last place) to euler roll, pitch, yaw.
    quat = [x, y, z, w]
    """
    x, y, z, w = quat
 
    
    t0 = 2.0*(w*x + y*z)
    t1=1.0-2.0*(x**2 + y**2)
    roll = atan2(t0, t1)
    
    t2=2.0*(w*y - z*x)
    pitch = asin(t2)
    
    t3=2.0*(w*z + x*y)
    t4=1.0-2.0*(y**2 + z**2)
    yaw = atan2(t3, t4)
    return roll, pitch, yaw



